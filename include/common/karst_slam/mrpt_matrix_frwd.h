#pragma once

#ifndef MRPT_MATRIX_FRWD_H
#define MRPT_MATRIX_FRWD_H

#include <cstddef>

// Forward declarations related to MRPT matrix types
namespace mrpt
{
    namespace math
    {
        template<class T, size_t N> class CArrayNumeric;
        template<class T> class CMatrixTemplateNumeric;
        template<class T, size_t N, size_t M> class CMatrixFixedNumeric;
        using CMatrixDouble22 = CMatrixFixedNumeric<double,2,2>;
        using CMatrixDouble33 = CMatrixFixedNumeric<double,3,3>;
        using CMatrixDouble44 = CMatrixFixedNumeric<double,4,4>;
        using CMatrixDouble55 = CMatrixFixedNumeric<double,5,5>;
        using CMatrixDouble66 = CMatrixFixedNumeric<double,6,6>;
    }
}

#endif //MRPT_MATRIX_FRWD_H
