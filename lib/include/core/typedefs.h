//
// Created by bene on 01.04.24.
//
#pragma once
#include <Eigen/Core>

#ifndef PRECISION
#define PRECSION double
#endif

namespace ukf {
namespace core {

template <int Size>
using Vector = Eigen::Vector<PRECSION, Size>;

template <int Rows, int Cols>
using Matrix = Eigen::Matrix<PRECSION, Rows, Cols>;

template <int Size>
using SquaredMatrix = Matrix<Size, Size>;

namespace math {

using SigmaPoints = Matrix<Eigen::Dynamic, Eigen::Dynamic>;
using TransformedSigmaPoints = Matrix<Eigen::Dynamic, Eigen::Dynamic>;

using DifferenceMatrix = Matrix<Eigen::Dynamic, Eigen::Dynamic>;

using CrossVariance = Matrix<Eigen::Dynamic, Eigen::Dynamic>;

using Mean = Vector<Eigen::Dynamic>;
using Covariance = SquaredMatrix<Eigen::Dynamic>;

}  // namespace math

namespace state {

using StateBase = Vector<Eigen::Dynamic>;
using CovarianceBase = SquaredMatrix<Eigen::Dynamic, Eigen::Dynamic>;

template <std::size_t N>
using FieldData = Vector<N>;

template <std::size_t N>
using FieldNoising = SquaredMatrix<N>;

using MeasurementType = Vector<Eigen::Dynamic>;
using NoisingType = SquaredMatrix<Eigen::Dynamic>

}  // namespace state

}  // namespace core
}  // namespace ukf
