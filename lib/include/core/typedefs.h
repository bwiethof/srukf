//
// Created by bene on 01.04.24.
//
#pragma once
#include <Eigen/Core>

#ifndef PRECISION
#define PRECISION double
#endif

namespace ukf {
namespace core {

using Float_t = PRECISION;
constexpr int DynamicSize = Eigen::Dynamic;

template <int Size>
using Vector = Eigen::Vector<Float_t, Size>;

template <int Rows, int Cols>
using Matrix = Eigen::Matrix<Float_t, Rows, Cols>;

template <int Size>
using SquaredMatrix = Matrix<Size, Size>;

template <std::size_t N>
using Array = std::array<Float_t, N>;

namespace math {
using SigmaPoints = Matrix<DynamicSize, DynamicSize>;
using TransformedSigmaPoints = Matrix<DynamicSize, DynamicSize>;

using DifferenceMatrix = Matrix<DynamicSize, DynamicSize>;

using CrossVariance = Matrix<DynamicSize, DynamicSize>;

using KalmanGain = Matrix<DynamicSize, DynamicSize>;

using Mean = Vector<DynamicSize>;
using Covariance = SquaredMatrix<DynamicSize>;

}  // namespace math

namespace state {

using StateBase = Vector<DynamicSize>;
using CovarianceBase = SquaredMatrix<DynamicSize>;

template <std::size_t N>
using FieldData = Vector<N>;

template <std::size_t N>
using FieldNoising = SquaredMatrix<N>;

using MeasurementType = Vector<DynamicSize>;
using NoisingType = SquaredMatrix<DynamicSize>;

}  // namespace state

}  // namespace core
}  // namespace ukf
