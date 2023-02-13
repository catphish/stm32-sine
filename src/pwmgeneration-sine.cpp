/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "pwmgeneration.h"
#include "hwdefs.h"
#include "params.h"
#include "inc_encoder.h"
#include "sine_core.h"
#include "fu.h"
#include "errormessage.h"
#include "digio.h"
#include "anain.h"
#include "my_math.h"
#include "picontroller.h"
#include "foc.h"

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)
#define INV_SQRT_1_5 FP_FROMFLT(0.8164965809)

void PwmGeneration::Run()
{
   if (opmode == MOD_RUN)
   {
      // Fetch settings
      int dir = Param::GetInt(Param::dir);
      s32fp throtcur  = Param::Get(Param::throtcur);
      s32fp curkp = Param::Get(Param::curkp);
      s32fp slipconst = Param::GetInt(Param::slipconst);

      // Update rotor angle from encoder
      Encoder::UpdateRotorAngle(dir);

      // Process currents
      s32fp il2 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
      s32fp il3 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));
      s32fp il1 = -il2 - il3;
      s32fp ilMax = GetIlMax(il1, il2);
      Param::SetFixed(Param::il1, il1);
      Param::SetFixed(Param::il2, il2);
      Param::SetFixed(Param::ilmax, ilMax);

      // Calculate measured IQ and ID
      FOC::SetAngle(angle);
      FOC::ParkClarke(il1, il2);
      Param::SetFixed(Param::iq, FOC::iq);
      Param::SetFixed(Param::id, FOC::id);

      // Calculate target IQ and ID
      s32fp idtarget = Param::Get(Param::idtarget);
      s32fp iqtarget = FP_MUL(throtcur, torqueRequest) * dir;

      // Increment rotor field angle
      int32_t slipIncr = 0;
      if(idtarget > 0)
      {
         slipIncr = iqtarget;
         slipIncr <<= 12;
         slipIncr /= slipconst;
         slipIncr <<= 12;
         slipIncr /= idtarget;
      }
      static uint32_t slipAngle = 0;
      slipAngle += slipIncr;
      slipAngle &= 0xFFFFFF;
      uint16_t rotorAngle = Encoder::GetRotorAngle();
      angle = -polePairRatio * rotorAngle + (slipAngle >> 8);
      Param::SetFixed(Param::fslipspnt, fslip);

      // Caculate slip frequency in Hz
      fslip = (slipIncr * pwmfrq) >> 19;
      Param::SetFixed(Param::fslipspnt, fslip);

      // Calculate rotor frequency in Hz
      frq = ABS(polePairRatio * Encoder::GetRotorFrequency() + fslip);
      Param::SetFixed(Param::fstat, frq);

      // Apply corrections to IQ and ID voltages
      static int32_t ud = 0;
      int32_t dError = idtarget - FOC::id;
      ud += FP_MUL(curkp, dError)/1000;
      if(ud > 26737) ud = 26737;
      if(ud < -26737) ud = -26737;
      static int32_t uq = 0;
      int32_t qError = iqtarget - FOC::iq;
      uq += FP_MUL(curkp, qError)/1000;
      if(uq > 26737) uq = 26737;
      if(uq < -26737) uq = -26737;
      Param::SetFixed(Param::ud, ud);
      Param::SetFixed(Param::uq, uq);

      // Inverse Park Clarke
      FOC::SetAngle(angle);
      FOC::InvParkClarke(ud, uq);

      /* Match to PWM resolution */
      timer_set_oc_value(PWM_TIMER, TIM_OC1, FOC::DutyCycles[0] >> shiftForTimer);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, FOC::DutyCycles[1] >> shiftForTimer);
      timer_set_oc_value(PWM_TIMER, TIM_OC3, FOC::DutyCycles[2] >> shiftForTimer);

      /* Shut down PWM if direction is zero */
      if (dir)
      {
         timer_enable_break_main_output(PWM_TIMER);
      }
      else
      {
         timer_disable_break_main_output(PWM_TIMER);
         ud = 0;
         uq = 0;
      }
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      AcHeat();
   }
}

void PwmGeneration::SetTorquePercent(float torque)
{
   // Set torque request, positive only
   torqueRequest = MAX(FP_FROMFLT(torque), 0);
}

void PwmGeneration::PwmInit()
{
   PwmGeneration::SetCurrentOffset(AnaIn::il1.Get(), AnaIn::il2.Get());
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   Encoder::SetPwmFrequency(pwmfrq);
   PwmGeneration::amp = 0;

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::GetIlMax(s32fp il1, s32fp il2)
{
   s32fp il3 = -il1 - il2;
   s32fp ilMax = fp_hypot3(il1, il2, il3);
   ilMax = FP_MUL(ilMax, INV_SQRT_1_5);

   return ilMax;
}
