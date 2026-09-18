#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

class ButterworthLowPass2
{
public:
    using Signal = Eigen::VectorXd;

    ButterworthLowPass2(double sampleRate,
                        double cutoffFrequency)
    : sampleRate_(sampleRate),
      cutoffFrequency_(cutoffFrequency)
    {
        if (sampleRate_ <= 0.0)
        {
            throw std::invalid_argument("ButterworthLowPass2: sample rate must be > 0");
        }

        setCutoffFrequency(cutoffFrequency_);
    }

    /**
     * Filter one input sample.
     *
     * The vector dimension can be arbitrary. The filter automatically
     * initializes its internal state to the correct size on first call.
     */
    Signal update(const Eigen::Ref<const Signal>& x)
    {
        if (x.size() == 0)
        {
            return x;
        }

        // Initialize / resize state when dimension changes.
        if (!initialized_ || stateSize_ != x.size())
        {
            z1_ = Signal::Zero(x.size());
            z2_ = Signal::Zero(x.size());

            initialized_ = true;
            stateSize_ = x.size();
        }

        // Transposed Direct Form II:
        //
        // y[n]  = b0*x[n] + z1
        // z1    = b1*x[n] - a1*y[n] + z2
        // z2    = b2*x[n] - a2*y[n]

        Signal y = b0_ * x + z1_;

        const Signal new_z1 =
            b1_ * x
            - a1_ * y
            + z2_;

        const Signal new_z2 =
            b2_ * x
            - a2_ * y;

        z1_ = new_z1;
        z2_ = new_z2;

        return y;
    }

    /**
     * Change cutoff frequency.
     *
     * The internal state is preserved.
     */
    void setCutoffFrequency(double cutoffFrequency)
    {
        if (cutoffFrequency <= 0.0)
        {
            throw std::invalid_argument(
                "ButterworthLowPass2: cutoff frequency must be > 0");
        }

        if (cutoffFrequency >= sampleRate_ * 0.5)
        {
            throw std::invalid_argument(
                "ButterworthLowPass2: cutoff frequency must be < Nyquist frequency");
        }

        cutoffFrequency_ = cutoffFrequency;

        updateCoefficients();
    }

    /**
     * Change sampling frequency and recompute coefficients.
     */
    void setSampleRate(double sampleRate)
    {
        if (sampleRate <= 0.0)
        {
            throw std::invalid_argument(
                "ButterworthLowPass2: sample rate must be > 0");
        }

        sampleRate_ = sampleRate;

        if (cutoffFrequency_ >= sampleRate_ * 0.5)
        {
            throw std::invalid_argument(
                "ButterworthLowPass2: cutoff frequency must be < Nyquist frequency");
        }

        updateCoefficients();
    }

    /**
     * Clear filter history.
     */
    void reset()
    {
        z1_.setZero();
        z2_.setZero();
        initialized_ = false;
        stateSize_ = 0;
    }

    double sampleRate() const
    {
        return sampleRate_;
    }

    double cutoffFrequency() const
    {
        return cutoffFrequency_;
    }

private:
    void updateCoefficients()
    {
        constexpr double Q = 1.0 / std::sqrt(2.0); // Butterworth Q

        const double omega = 2.0 * M_PI * cutoffFrequency_ / sampleRate_;

        const double cosOmega = std::cos(omega);
        const double sinOmega = std::sin(omega);

        const double alpha = sinOmega / (2.0 * Q);

        // RBJ low-pass biquad coefficients
        const double b0 = (1.0 - cosOmega) * 0.5;
        const double b1 =  1.0 - cosOmega;
        const double b2 = (1.0 - cosOmega) * 0.5;

        const double a0 = 1.0 + alpha;
        const double a1 = -2.0 * cosOmega;
        const double a2 = 1.0 - alpha;

        // Normalize by a0 so that a0 = 1.
        b0_ = b0 / a0;
        b1_ = b1 / a0;
        b2_ = b2 / a0;

        a1_ = a1 / a0;
        a2_ = a2 / a0;
    }

private:
    double sampleRate_{1000.0};
    double cutoffFrequency_{10.0};

    // Biquad coefficients
    double b0_{};
    double b1_{};
    double b2_{};
    double a1_{};
    double a2_{};

    // Filter state
    Signal z1_;
    Signal z2_;

    Eigen::Index stateSize_{0};
    bool initialized_{false};
};