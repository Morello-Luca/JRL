#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cstddef>

template <std::size_t N>
class MedianFilter
{
public:
    using Signal = Eigen::VectorXd;

    Signal update(const Eigen::Ref<const Signal>& x)
    {
        // Initialize on first sample
        if (!initialized_)
        {
            for (auto& sample : buffer_)
            {
                sample = x;
            }

            initialized_ = true;
            dimension_ = x.size();
        }

        if (x.size() != dimension_)
        {
            throw std::runtime_error(
                "MedianFilter: input signal dimension changed.");
        }

        buffer_[index_] = x;
        index_ = (index_ + 1) % N;

        Signal result(dimension_);

        // Compute median independently for each signal component
        for (int i = 0; i < dimension_; ++i)
        {
            std::array<double, N> values;

            for (std::size_t k = 0; k < N; ++k)
            {
                values[k] = buffer_[k](i);
            }

            std::sort(values.begin(), values.end());

            result(i) = values[N / 2];
        }

        return result;
    }

private:
    std::array<Signal, N> buffer_;
    std::size_t index_{0};
    int dimension_{0};
    bool initialized_{false};
};