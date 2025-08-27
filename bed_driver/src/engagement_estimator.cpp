// engagement_geom_estimator.h
#pragma once
#include <Arduino.h>
#include <math.h>

/**
 * Geometry-informed engagement estimator.
 *
 * Models (with optional bias b):
 *   y' = y - b
 *   H1 (engaged, known k0):        y' ~ k0 * x + N(0, sigma^2)
 *   H1 (engaged, soft prior k~N):  y' ~ k   * x + N(0, sigma^2),  k ~ N(k0,
 * tau^2) H0 (disengaged):               y' ~ 0   + N(0, sigma^2)
 *
 * Streaming evidence uses exponentially-weighted (EW) sums:
 *   Sxx = Σ λ x^2,  Sxy = Σ λ x y',  Syy = Σ λ y'^2
 *
 * LLR (evidence for H1 vs H0):
 *  - Fixed k0:   LLR = (1/(2*sigma^2)) * ( 2*k0*Sxy - k0^2*Sxx )
 *  - Soft prior: LLR = (1/(2*sigma^2)) * ( (Sxy'² / Sxx') ), with
 *                Sxx' = Sxx + lambda_k,  Sxy' = Sxy + lambda_k * k0,
 *                lambda_k = sigma^2 / tau^2
 *
 * Then Bayes factor B = exp(LLR), sticky prior, -> probability.
 */
class EngagementGeomEstimator
{
 public:
  // --- Construction ---
  // dt_s: loop period (s), Tc_s: evidence memory (s)
  EngagementGeomEstimator(float dt_s = 0.01f, float Tc_s = 0.8f)
  {
    configure(dt_s, Tc_s);
    reset(0.5f);
  }

  // --- Configuration ---
  void configure(float dt_s, float Tc_s)
  {
    dt_ = max(1e-4f, dt_s);
    Tc_ = max(0.05f, Tc_s);
    lambda_ = expf(-dt_ / Tc_);
    setBiasTimeConstant(max(0.5f, 2.5f * Tc_));  // bias slower than evidence
  }

  // Expected geometry gain k0 (rad/s per unit command); must set!
  void setK0(float k0_rad_per_cmd) { k0_ = k0_rad_per_cmd; }

  // Soft prior on gain: tau>0 enables ridge around k0, tau<=0 disables (fixed
  // k0)
  void setGainSoftPrior(float tau_std) { tau_ = tau_std; }

  // Noise variance (rad^2/s^2). If autoNoise=true, we update during quiet.
  void setNoiseSigma(float sigma_rad_per_s)
  {
    sigma2_ = sigma_rad_per_s * sigma_rad_per_s;
  }
  void setAutoNoise(bool on) { autoNoise_ = on; }

  // Bias settings: track gyro bias only when |x| < x_quiet
  void setBiasTimeConstant(float Tb_s)
  {
    Tb_ = max(0.1f, Tb_s);
    lambda_b_ = expf(-dt_ / Tb_);
  }
  void setQuietThreshold(float x_abs) { x_quiet_ = max(0.0f, x_abs); }

  // Sticky prior (prevents flicker)
  void setStickiness(float p11_keep, float p01_enter)
  {
    p11_ = constrain(p11_keep, 0.0f, 1.0f);
    p01_ = constrain(p01_enter, 0.0f, 1.0f);
  }

  // Decision hysteresis (probability thresholds)
  void setHysteresis(float on, float off)
  {
    h_on_ = constrain(on, 0.0f, 1.0f);
    h_off_ = constrain(off, 0.0f, 1.0f);
  }

  // Evidence & numeric guards
  void setMemory(float Tc_s) { configure(dt_, Tc_s); }
  void setDt(float dt_s) { configure(dt_s, Tc_); }
  void setInputEnergyFloor(float f) { Sxx_floor_ = max(1e-12f, f); }
  void setVarFloorFrac(float frac) { varFloorFrac_ = max(1e-9f, frac); }
  void setLLRClamp(float lo, float hi)
  {
    llr_min_ = lo;
    llr_max_ = hi;
  }

  // Reset
  void reset(float prior_engaged = 0.5f)
  {
    Sxx_ = Sxy_ = Syy_ = 0.0f;
    pi_ = constrain(prior_engaged, 0.0f, 1.0f);
    engaged_ = (pi_ > h_on_);
    b_ = 0.0f;
    // default conservative noise if not set yet
    if (!(sigma2_ > 0.0f)) sigma2_ = 1e-4f;  // (0.01 rad/s)^2
  }

  // --- Main update ---
  // x: command (unitless, e.g., power in [-1,1]); y: gyro rate (rad/s)
  // Returns updated engagement probability in [0,1].
  float update(float x, float y)
  {
    // 1) Update bias when command is quiet
    const bool quiet = (fabsf(x) <= x_quiet_);
    if (quiet)
    {
      b_ = lambda_b_ * b_ + (1.0f - lambda_b_) * y;
      // optional: update noise using y' during quiet
      if (autoNoise_)
      {
        const float yq = y - b_;
        // simple EW variance about zero under H0
        sigma2_ = 0.98f * sigma2_ + 0.02f * (yq * yq + 1e-12f);
      }
    }

    // 2) Residual relative to bias
    const float yp = y - b_;  // y'

    // 3) EW streaming stats (no centering -> retain DC info)
    Sxx_ = lambda_ * Sxx_ + x * x;
    Sxy_ = lambda_ * Sxy_ + x * yp;
    Syy_ = lambda_ * Syy_ + yp * yp;

    // 4) Compute LLR (evidence for engagement)
    float LLR = 0.0f;

    const bool have_input = (Sxx_ > Sxx_floor_);
    if (have_input && sigma2_ > 0.0f)
    {
      if (tau_ > 0.0f)
      {
        // Soft prior on k: ridge around k0
        const float lambda_k = sigma2_ / (tau_ * tau_);  // strength of prior
        const float Sxx_p = Sxx_ + lambda_k;
        const float Sxy_p = Sxy_ + lambda_k * k0_;
        // LLR = (1/(2*sigma^2)) * ((Sxy')^2 / Sxx')
        const float num = Sxy_p * Sxy_p;
        const float den = max(Sxx_p, 1e-12f);
        float llr = 0.5f * (num / den) / sigma2_;
        LLR = constrain(llr, llr_min_, llr_max_);
        khat_MAP_ = Sxy_p / den;  // MAP estimate of k
        // If geometry says k must be >=0 and estimate goes negative, zero
        // evidence
        if (enforcePositiveK_ && khat_MAP_ < 0.0f) LLR = 0.0f;
      }
      else
      {
        // Fixed k0 (fastest): LLR = (1/(2*sigma^2)) * (2*k0*Sxy - k0^2*Sxx)
        float llr = (k0_ * (2.0f * Sxy_ - k0_ * Sxx_)) / (2.0f * sigma2_);
        LLR = constrain(llr, llr_min_, llr_max_);
        khat_MAP_ = k0_;
      }
    }
    else
    {
      // No recent input energy -> neutral evidence
      LLR = 0.0f;
    }

    // 5) Bayes update with sticky prior
    const float BF = expf(LLR);
    // const float pi_pred = p01_ * (1.0f - pi_) + p11_ * pi_;
    float pi_pred;
    if (fabsf(x) <= x_quiet_)
    {
      pi_pred = pi_;  // freeze: no spontaneous switching
    }
    else
    {
      pi_pred =
          p01_ * (1.0f - pi_) + p11_ * pi_;  // allow switching when excited
    }
    const float num = BF * pi_pred;
    const float den = num + (1.0f - pi_pred);
    if (den > 0.0f) pi_ = num / den;
    pi_ = constrain(pi_, 0.0f, 1.0f);

    // 6) Hysteretic boolean decision
    if (!engaged_ && pi_ > h_on_) engaged_ = true;
    if (engaged_ && pi_ < h_off_) engaged_ = false;

    return pi_;
  }

  // --- Accessors / telemetry ---
  float prob() const { return pi_; }
  bool engaged() const { return engaged_; }
  float bias() const { return b_; }
  float noiseSigma() const { return sqrtf(sigma2_); }
  float k0() const { return k0_; }
  float kMAP() const { return khat_MAP_; }  // equal to k0 if tau<=0
  float Sxx() const { return Sxx_; }
  float Sxy() const { return Sxy_; }
  float Syy() const { return Syy_; }
  void enforcePositiveK(bool on) { enforcePositiveK_ = on; }

 private:
  // Time constants
  float dt_ = 0.01f, Tc_ = 0.8f;
  float lambda_ = expf(-0.01f / 0.8f);

  // Geometry & noise
  float k0_ = 0.1f;       // set me!
  float tau_ = 0.0f;      // 0 => fixed k0, >0 => soft prior std dev
  float sigma2_ = 1e-4f;  // (0.01 rad/s)^2
  bool autoNoise_ = true;
  bool enforcePositiveK_ = true;

  // Bias tracking
  float b_ = 0.0f;
  float Tb_ = 2.0f;  // bias time constant (s)
  float lambda_b_ = expf(-0.01f / 2.0f);
  float x_quiet_ = 0.03f;  // |x| < this -> update bias

  // EW stats
  float Sxx_ = 0.0f, Sxy_ = 0.0f, Syy_ = 0.0f;

  // Sticky prior & hysteresis
  float p11_ = 0.98f, p01_ = 0.02f;
  float h_on_ = 0.90f, h_off_ = 0.60f;

  // Numerics
  float Sxx_floor_ = 1e-9f;
  float varFloorFrac_ =
      1e-6f;  // (kept for extension; not needed in current LLR forms)
  float llr_min_ = -40.0f, llr_max_ = 40.0f;

  // Diagnostics
  float pi_ = 0.5f;
  bool engaged_ = false;
  float khat_MAP_ = 0.0f;
};