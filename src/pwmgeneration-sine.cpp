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

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)
#define INV_SQRT_1_5 FP_FROMFLT(0.8164965809)

void PwmGeneration::Run()
{
   if (opmode == MOD_MANUAL || opmode == MOD_RUN)
   {
      static int32_t amp = 0;
      int dir = Param::GetInt(Param::dir);

      Encoder::UpdateRotorAngle(dir);
      ProcessCurrents();
      CalcNextAngleAsync();

      // Adjust amplitude according to requested and measured current
      s32fp curkp = Param::Get(Param::curkp);
      s32fp ilmax = Param::Get(Param::ilmax);
      s32fp throtcur = Param::Get(Param::throtcur);
      s32fp ilmaxtarget = FP_MUL(throtcur, ampnom);
      s32fp error = ilmaxtarget - ilmax;
      s32fp correction = FP_MUL(error, curkp);

      // Apply a correction to the amplitude
      amp += correction;

      if (amp > 16777215)
         amp = 16777215;
      else if (amp < 0)
         amp = 0;

      SineCore::SetAmp(amp >> 9);
      Param::SetInt(Param::amp, amp >> 9);
      Param::SetFixed(Param::fstat, frq);
      Param::SetFixed(Param::angle, DIGIT_TO_DEGREE(angle));
      SineCore::Calc(angle);

      /* Shut down PWM on zero voltage request */
      if (0 == amp || 0 == dir)
      {
         timer_disable_break_main_output(PWM_TIMER);
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      /* Match to PWM resolution */
      timer_set_oc_value(PWM_TIMER, TIM_OC1, SineCore::DutyCycles[0] >> shiftForTimer);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[1] >> shiftForTimer);
      timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[2] >> shiftForTimer);
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
   float fslipmax = Param::GetFloat(Param::fslipmax);

   ampnom = FP_FROMFLT(ABS(torque));
   fslip = FP_FROMFLT(torque / 100.f * fslipmax);

   Param::Set(Param::ampnom, ampnom);
   Param::Set(Param::fslipspnt, fslip);

   slipIncr = FRQ_TO_ANGLE(fslip);
}

void PwmGeneration::PwmInit()
{
   PwmGeneration::SetCurrentOffset(AnaIn::il1.Get(), AnaIn::il2.Get());
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   Encoder::SetPwmFrequency(pwmfrq);

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::GetIlMax(s32fp il1, s32fp il2)
{
   s32fp il3 = -il1 - il2;
   s32fp ilMax = FP_MUL(il1, il1) + FP_MUL(il2, il2) + FP_MUL(il3, il3);
   ilMax = fp_sqrt(ilMax);
   ilMax = FP_MUL(ilMax, INV_SQRT_1_5);

   return ilMax;
}

s32fp PwmGeneration::ProcessCurrents()
{
   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

   s32fp ilMax = GetIlMax(il1, il2);

   Param::SetFixed(Param::il1, il1);
   Param::SetFixed(Param::il2, il2);
   Param::SetFixed(Param::ilmax, ilMax);

   return ilMax;
}
