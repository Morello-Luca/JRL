#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <type_traits>
#include <vector>

#include <SpaceVecAlg/SpaceVecAlg>

// Requires C++17 (if constexpr, std::void_t).

namespace hampel_detail
{

// ---------------------------------------------------------------------
// VectorTraits: lets HampelFilter work on scalars, Eigen-like vectors,
// and SpaceVecAlg types (ForceVec, MotionVec) through one interface.
// Add a specialization here for any other type you want to filter.
// ---------------------------------------------------------------------

// Primary template: treated as a plain scalar (double, float, ...).
template<typename T, typename = void>
struct VectorTraits
{
  static constexpr bool is_scalar = true;

  static std::size_t size(const T &) { return 1; }

  static void toArray(const T & v, double * out) { out[0] = static_cast<double>(v); }

  static void fromArray(T & v, const double * in) { v = static_cast<T>(in[0]); }
};

// Anything exposing .size() and operator[] (Eigen::VectorXd, std::vector<double>, ...).
template<typename T>
struct VectorTraits<T,
                     std::void_t<decltype(std::declval<const T &>().size()),
                                 decltype(std::declval<const T &>()[std::size_t{0}])>>
{
  static constexpr bool is_scalar = false;

  static std::size_t size(const T & v) { return static_cast<std::size_t>(v.size()); }

  static void toArray(const T & v, double * out)
  {
    for(std::size_t i = 0; i < size(v); ++i)
    {
      out[i] = static_cast<double>(v[i]);
    }
  }

  static void fromArray(T & v, const double * in)
  {
    for(std::size_t i = 0; i < size(v); ++i)
    {
      v[i] = static_cast<typename std::decay<decltype(v[0])>::type>(in[i]);
    }
  }
};

// sva::ForceVec<double>: couple + force, 6 components.
template<>
struct VectorTraits<sva::ForceVec<double>>
{
  static constexpr bool is_scalar = false;

  static std::size_t size(const sva::ForceVec<double> &) { return 6; }

  static void toArray(const sva::ForceVec<double> & v, double * out)
  {
    const Eigen::Vector6d raw = v.vector();
    for(int i = 0; i < 6; ++i)
    {
      out[i] = raw[i];
    }
  }

  static void fromArray(sva::ForceVec<double> & v, const double * in)
  {
    Eigen::Vector6d raw;
    for(int i = 0; i < 6; ++i)
    {
      raw[i] = in[i];
    }
    v = sva::ForceVec<double>(raw);
  }
};

// sva::MotionVec<double>: angular + linear velocity, 6 components.
template<>
struct VectorTraits<sva::MotionVec<double>>
{
  static constexpr bool is_scalar = false;

  static std::size_t size(const sva::MotionVec<double> &) { return 6; }

  static void toArray(const sva::MotionVec<double> & v, double * out)
  {
    const Eigen::Vector6d raw = v.vector();
    for(int i = 0; i < 6; ++i)
    {
      out[i] = raw[i];
    }
  }

  static void fromArray(sva::MotionVec<double> & v, const double * in)
  {
    Eigen::Vector6d raw;
    for(int i = 0; i < 6; ++i)
    {
      raw[i] = in[i];
    }
    v = sva::MotionVec<double>(raw);
  }
};

} // namespace hampel_detail

/*
 * Generic Hampel filter.
 *
 * Works out of the box on:
 *   - plain scalars (double, float, ...)
 *   - anything with .size() / operator[] (Eigen::VectorXd, std::vector<double>, ...)
 *   - sva::ForceVec<double> and sva::MotionVec<double>
 *
 * Each component of the input is filtered independently against the
 * median of its own recent (cleaned) history, using the Median Absolute
 * Deviation (MAD) as a robust estimate of spread. To support another
 * type, add a hampel_detail::VectorTraits<YourType> specialization.
 *
 * Example:
 *
 *   HampelFilter hampel(5, 3.0, 0.1);
 *   sva::ForceVec<double> cleanedWrench = hampel.update(rawWrench);
 *
 *   HampelFilter jointHampel(5, 3.0, 0.01);
 *   Eigen::VectorXd cleanedQ = jointHampel.update(rawQ);
 */
class HampelFilter
{
public:
  enum class OutlierAction
  {
    ReplaceWithMedian, // snap outliers to the local median (default, matches original behaviour)
    Clamp // pull outliers back to the edge of the allowed band instead of fully discarding them
  };

  /*
   * windowSize:    number of previous (cleaned) samples used for the local median/MAD
   * threshold:     Hampel multiplier; larger -> less aggressive rejection
   * minDeviation:  floor on the allowed deviation, so a near-constant signal
   *                doesn't start rejecting everything once MAD collapses to ~0
   * action:        what to do with a detected outlier
   */
  explicit HampelFilter(std::size_t windowSize = 5,
                         double threshold = 3.0,
                         double minDeviation = 0.1,
                         OutlierAction action = OutlierAction::ReplaceWithMedian)
  : windowSize_(std::max(windowSize, std::size_t{3})), threshold_(std::max(threshold, 0.0)),
    minDeviation_(std::max(minDeviation, 0.0)), action_(action)
  {
  }

  template<typename VectorT>
  VectorT update(const VectorT & input)
  {
    using Traits = hampel_detail::VectorTraits<VectorT>;
    const std::size_t dim = Traits::size(input);

    if(histories_.size() != dim)
    {
      histories_.clear();
      histories_.resize(dim);
      outlierMask_.assign(dim, false);
    }

    if constexpr(Traits::is_scalar)
    {
      const double raw = static_cast<double>(input);
      const double cleaned = updateScalar(raw, histories_[0], outlierMask_[0]);
      return static_cast<VectorT>(cleaned);
    }
    else
    {
      scratchRaw_.resize(dim);
      scratchClean_.resize(dim);

      Traits::toArray(input, scratchRaw_.data());

      for(std::size_t i = 0; i < dim; ++i)
      {
        scratchClean_[i] = updateScalar(scratchRaw_[i], histories_[i], outlierMask_[i]);
      }

      VectorT output = input;
      Traits::fromArray(output, scratchClean_.data());
      return output;
    }
  }

  /*
   * Whether the last update() classified each component as an outlier
   * (index 0 for scalar inputs). Handy for logging/diagnostics.
   */
  const std::deque<bool> & lastOutlierMask() const { return outlierMask_; }

  /*
   * Clear all stored histories (and outlier flags).
   */
  void reset()
  {
    for(auto & history : histories_)
    {
      history.clear();
    }
    std::fill(outlierMask_.begin(), outlierMask_.end(), false);
  }

private:
  /*
   * Process one scalar channel.
   *
   * The current sample is compared against PREVIOUS VALID (cleaned)
   * samples. If it's an outlier it's replaced (or clamped) according to
   * `action_`; the CLEANED value is what gets stored in the history, so
   * an outlier can't contaminate future statistics.
   */
  double updateScalar(double x, std::deque<double> & history, bool & outlierFlag)
  {
    outlierFlag = false;

    // Not enough history yet for a meaningful Hampel test: accept as-is.
    if(history.size() < 3)
    {
      history.push_back(x);
      if(history.size() > windowSize_)
      {
        history.pop_front();
      }
      return x;
    }

    // Local median of the previous cleaned samples.
    scratchMedianBuf_.assign(history.begin(), history.end());
    const double median = computeMedian(scratchMedianBuf_);

    // Median Absolute Deviation (MAD) -> robust sigma estimate.
    scratchMedianBuf_.resize(history.size());
    {
      std::size_t idx = 0;
      for(const double sample : history)
      {
        scratchMedianBuf_[idx++] = std::abs(sample - median);
      }
    }
    const double mad = computeMedian(scratchMedianBuf_);
    const double sigma = 1.4826 * mad;

    const double allowedDeviation = std::max(threshold_ * sigma, minDeviation_);
    const double distance = std::abs(x - median);

    double cleanedValue = x;

    if(distance > allowedDeviation)
    {
      outlierFlag = true;
      cleanedValue = (action_ == OutlierAction::ReplaceWithMedian)
                         ? median
                         : median + std::copysign(allowedDeviation, x - median);
    }

    history.push_back(cleanedValue);
    if(history.size() > windowSize_)
    {
      history.pop_front();
    }

    return cleanedValue;
  }

  /*
   * Median via nth_element: O(n) average instead of the O(n log n) a full
   * sort costs. Mutates `values`, which is fine since callers only ever
   * pass a scratch buffer.
   */
  static double computeMedian(std::vector<double> & values)
  {
    const std::size_t n = values.size();
    const std::size_t mid = n / 2;

    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double median = values[mid];

    if(n % 2 == 0)
    {
      std::nth_element(values.begin(), values.begin() + mid - 1, values.begin() + mid);
      median = 0.5 * (values[mid - 1] + median);
    }

    return median;
  }

  std::size_t windowSize_;
  double threshold_;
  double minDeviation_;
  OutlierAction action_;

  std::vector<std::deque<double>> histories_;
  std::deque<bool> outlierMask_; // deque<bool> isn't bit-packed like vector<bool>, so operator[] yields a real bool&

  // Reused scratch buffers so update() doesn't allocate on every call.
  std::vector<double> scratchRaw_;
  std::vector<double> scratchClean_;
  std::vector<double> scratchMedianBuf_;
};