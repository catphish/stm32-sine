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
   if (opmode == MOD_RUN)
   {
      // Process currents and get ilmax
      ProcessCurrents();
      s32fp ilmax = Param::Get(Param::ilmax);

      // amp is the amplitude of the sine wave output
      // This value persists and is modified each cycle
      static int32_t amp = 0;

      // Get requested direction from DNR switch
      int dir = Param::GetInt(Param::dir);

      // If torque is less than zero, we are in regen mode
      int regen = torqueRequest < 0;

      // Disable regen if we are not travelling in the requested direction
      if (regen && Encoder::GetRotorDirection() != Param::GetInt(Param::dir))
         torqueRequest = 0;

      s32fp fslipmax;
      s32fp fslipmin;
      s32fp throtcur;
      // If regen is requested, use "r" parameters
      if (regen)
      {
         fslipmax = -Param::Get(Param::rfslipmax);
         fslipmin = -Param::Get(Param::rfslipmin);
         throtcur = Param::Get(Param::rthrotcur);
      }
      // If torque is positive use "m" parameters
      else
      {
         fslipmax = Param::Get(Param::mfslipmax);
         fslipmin = Param::Get(Param::mfslipmin);
         throtcur = Param::Get(Param::throtcur);
      }

      // Set ampnom to magnitude of torque request
      ampnom = ABS(torqueRequest);

      // Set slip according to torque request and fslipmax
      fslip = fslipmin + FP_MUL(ampnom, (fslipmax - fslipmin)) / 100;

      // Set parameters for logging
      Param::SetFixed(Param::ampnom, ampnom);
      Param::SetFixed(Param::fslipspnt, fslip);

      // Set slip increment angle
      slipIncr = FRQ_TO_ANGLE(fslip * dir);

      // Calculate current angle
      Encoder::UpdateRotorAngle(dir);
      CalcNextAngleAsync();

      // Adjust amplitude according to requested and measured current
      s32fp curkp = Param::Get(Param::curkp);

      s32fp ilmaxtarget = FP_MUL(throtcur, ampnom);
      Param::SetFixed(Param::ilmaxtarget, ilmaxtarget);

      // Calculate DC current
      s32fp idc = ilmax * SineCore::GetAmp() / SineCore::MAXAMP;
      idc = FP_MUL(idc, FP_FROMFLT(1.2247)); // Multiply by sqrt(3), divide by sqrt(2)
      Param::SetFixed(Param::idc, idc);

      // Apply a correction to the amplitude
      s32fp ampslew = Param::Get(Param::ampslew);
      s32fp ierror = ilmaxtarget - ilmax;
      s32fp correction = FP_MUL(ierror, curkp);
      correction = MIN(correction, ampslew);
      correction = MAX(correction, -ampslew);
      amp += correction;

      // Limit amplitude to 0..MAXAMP, shift by 9 bits to get more resolution
      int32_t maxamp = ((SineCore::MAXAMP << 9) | 0x1FF);
      if (amp > maxamp)
         amp = maxamp;
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
   PwmGeneration::torqueRequest = FP_FROMFLT(torque);
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
