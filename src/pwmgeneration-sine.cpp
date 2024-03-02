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

      // Get requested direction from DNR switch
      int dir = Param::GetInt(Param::dir);

      // Fetch settings
      s32fp fweakmin  = Param::Get(Param::fweakmin);
      s32fp fweakmax  = Param::Get(Param::fweakmax);
      s32fp fslipweak = Param::Get(Param::fslipweak);
      s32fp fslipmax  = Param::Get(Param::fslipmax);
      s32fp fslipmin  = Param::Get(Param::fslipmin);
      s32fp throtcur  = Param::Get(Param::throtcur);
      // Fetch current correction gain
      s32fp curkp     = Param::Get(Param::curkp);
      // Fetch regen V/Hz curve.
      int32_t vhzregen = Param::GetInt(Param::vhzregen);

      if (torqueRequest < 0) { // If torque is negative, do regen
         // Set slip betwen 0 and fslipmax. This is negative because torqueRequest is negative
         fslip = FP_MUL(fslipmax, torqueRequest) / 100;

         // Do an extra local calculation of the electrical frequency adding the (negative) slip
         s32fp lfrq = polePairRatio * Encoder::GetRotorFrequency();
         lfrq += fslip;
         if(lfrq < 0) lfrq = 0;

         // Set amp using a fixed V/Hz ratio
         amp = lfrq * vhzregen; // A value of 3025 gives full voltage at 200Hz

         // Log current target as zero for logging only
         Param::SetFixed(Param::ilmaxtarget, 0);
      } else { // If torque is positive, do active current control
         // Set ampnom to magnitude of torque request
         ampnom = ABS(torqueRequest);

         // Calculate target current
         s32fp ilmaxtarget = FP_MUL(throtcur, ampnom);

         // Calculate alternative current limit based on DC current
         // Ignore if voltage is near zero
         if(SineCore::GetAmp() > 64)
         {
            s32fp ilmaxtargetdc = Param::GetInt(Param::idcmax) * SineCore::MAXAMP / SineCore::GetAmp() * 32;
            if (ilmaxtargetdc < ilmaxtarget) ilmaxtarget = ilmaxtargetdc;
         }

         // Save current target for logging
         Param::SetFixed(Param::ilmaxtarget, ilmaxtarget);

         // Calculate current error
         s32fp ierror = ilmaxtarget - ilmax;
         // Calculate and apply voltage correction
         s32fp correction = (ierror * curkp) / 4096;
         amp += correction;

         // If field weakening is configured, calculate a new fslipmax accordingly
         if (fslipweak > fslipmax && fweakmax > fweakmin) {
            s32fp frq = polePairRatio * Encoder::GetRotorFrequency();
            if(frq > fweakmax)
               // If frequency is above fweakmax, use 100% fslipweak
               fslipmax = fslipweak;
            else if (frq > fweakmin) {
               // If frequency is between fweakmin and fweakmax, interpolate between fslipmax and fslipweak
               fslipmax = fslipmax + FP_DIV(FP_MUL(fslipweak - fslipmax, frq - fweakmin), (fweakmax - fweakmin));
            } // Otherwise fslipmax is used
         }

         // Set slip according to torque request and fslipmax
         fslip = fslipmin + FP_MUL(ampnom, (fslipmax - fslipmin)) / 100;
      }

      // Limit amplitude to 0..MAXAMP, shift by 9 bits to get more resolution
      int32_t maxamp = (SineCore::MAXAMP << 9) | 0x1FF;
      if (amp > maxamp)
         amp = maxamp;
      else if (amp < 0)
         amp = 0;

      // Set parameters for logging
      Param::SetFixed(Param::ampnom, ampnom);
      Param::SetFixed(Param::fslipspnt, fslip);

      // Set slip increment angle
      slipIncr = FRQ_TO_ANGLE(fslip * dir);

      // Calculate current angle
      Encoder::UpdateRotorAngle(dir);
      CalcNextAngleAsync();

      // Use SineCore module to calulate PWM duty cycles
      SineCore::SetAmp(amp >> 9);
      Param::SetInt(Param::amp, amp >> 9);
      Param::SetFixed(Param::fstat, frq);
      Param::SetFixed(Param::angle, DIGIT_TO_DEGREE(angle));
      SineCore::Calc(angle);

      /* Shut down PWM if amplitude or direction are zero */
      if (0 == amp || 0 == dir)
      {
         timer_disable_break_main_output(PWM_TIMER);
         amp = 0;
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
   // Don't allow regen to accelerate backwards
   if (Encoder::GetRotorDirection() != Param::GetInt(Param::dir) && torque < 0)
      torque = 0;
   // Set torque request
   torqueRequest = FP_FROMFLT(torque);
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

s32fp PwmGeneration::ProcessCurrents()
{
   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

   s32fp ilMax = GetIlMax(il1, il2);

   Param::SetFixed(Param::il1, il1);
   Param::SetFixed(Param::il2, il2);
   Param::SetFixed(Param::ilmax, ilMax);

   // Multiply AC current by DC voltage fraction to get DC current
   s32fp idc = ilMax * SineCore::GetAmp() / SineCore::MAXAMP;
   // In theory we should divide by sqrt(2) to convert to RMS
   // and then multiply by sqrt(3) to convert to single phase
   // We can just multiply by 1.2247 to get the same result
   // but I'm not going to bother because the result is not
   // scaled correctly anyway.
   // idc = FP_MUL(idc, FP_FROMFLT(1.2247));
   Param::SetFixed(Param::idc, idc);

   return ilMax;
}
