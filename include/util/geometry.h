#pragma once
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>



template <size_t row, size_t col, typename T>
class Matrix;

template <size_t d, typename T>
struct Vector {
    Vector() {
        for (size_t i = 0; i < d; ++i) raw[i] = T();
    }
    T& operator[](const size_t& i) {
        assert(i < d);
        return raw[i];
    }
    const T& operator[](const size_t& i) const {
        assert(i < d);
        return raw[i];
    }

    Vector<d, T> round() {
        Vector<d, T> ret;
        for (size_t i = 0; i < d; ++i) ret[i] = std::round(raw[i]);
        return ret;
    }

   private:
    T raw[d];
};

template <typename T>
struct Vector<2, T> {
    T x, y;
    Vector() : x(T()), y(T()) {}
    Vector(const T& t_x, const T& t_y) : x(t_x), y(t_y) {}
    template <typename U>
    Vector(const Vector<2, U>& other) : x(other.x), y(other.y){};

    T& operator[](const size_t& i) {
        assert(i < 2);
        return i == 0 ? x : y;
    }
    const T& operator[](const size_t& i) const {
        assert(i < 2);
        return i == 0 ? x : y;
    }

    float norm() { return std::sqrt(x * x + y * y); }
    Vector<3, T>& normalize(T l = 1) {
        *this = (*this) * (l / norm());
        return *this;
    }
};

template <typename T>
struct Vector<3, T> {
    T x, y, z;
    Vector() : x(T()), y(T()), z(T()) {}
    Vector(const T& t_x, const T& t_y, const T& t_z) : x(t_x), y(t_y), z(t_z) {}
    template <typename U>
    Vector(const Vector<3, U>& other) : x(other.x), y(other.y), z(other.z){};

    T& operator[](const size_t& i) {
        assert(i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }
    const T& operator[](const size_t& i) const {
        assert(i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }

    float norm() { return std::sqrt(x * x + y * y + z * z); }
    Vector<3, T>& normalize(T l = 1) {
        *this = (*this) * (l / norm());
        return *this;
    }

    Vector<3, T> round() { return {std::round(x), std::round(y), std::round(z)}; }
};

template <size_t d, typename T>
T operator*(const Vector<d, T>& lhs, const Vector<d, T>& rhs) {
    T ret = T();
    for (size_t i = 0; i < d; ++i) ret += lhs[i] * rhs[i];
    return ret;
}

template <size_t d, typename T>
Vector<d, T> operator+(const Vector<d, T>& lhs, const Vector<d, T>& rhs) {
    Vector<d, T> ret;
    for (size_t i = 0; i < d; ++i) ret[i] = lhs[i] + rhs[i];
    return ret;
}

template <size_t d, typename T>
Vector<d, T> operator-(const Vector<d, T>& lhs, const Vector<d, T>& rhs) {
    Vector<d, T> ret;
    for (size_t i = 0; i < d; ++i) ret[i] = lhs[i] - rhs[i];
    return ret;
}

template <size_t d, typename T, typename U>
Vector<d, T> operator*(const Vector<d, T>& lhs, const U& rhs) {
    Vector<d, T> ret;
    for (size_t i = 0; i < d; ++i) ret[i] = lhs[i] * rhs;
    return ret;
}

template <size_t d, typename T, typename U>
Vector<d, T> operator/(const Vector<d, T>& lhs, const U& rhs) {
    Vector<d, T> ret;
    for (size_t i = 0; i < d; ++i) ret[i] = lhs[i] / rhs;
    return ret;
}

template <size_t l, size_t d, typename T>
Vector<l, T> embed(const Vector<d, T>& v, T fill = 1) {
    Vector<l, T> ret;
    for (size_t i = 0; i < l; ++i) ret[i] = (i < d ? v[i] : fill);
    return ret;
}

template <size_t l, size_t d, typename T>
Vector<l, T> projection(const Vector<d, T>& v) {
    Vector<l, T> ret;
    for (size_t i = 0; i < l; ++i) ret[i] = v[i];
    return ret;
}

template <typename T>
Vector<3, T> cross(const Vector<3, T>& lhs, const Vector<3, T>& rhs) {
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

template <size_t d, typename T>
std::ostream operator<<(std::ostream& s, Vector<d, T>& v) {
    s << "(";
    for (size_t i = 0; i < d; ++i) s << v[i] << (i == d - 1 ? ")" : ", ");
    return s;
}

template <size_t d, typename T>
struct dt {
    static T det(const Matrix<d, d, T>& m) {
        T ans = 0;
        for (size_t i = 0; i < d; ++i) ans += m[0][i] * m.cofactor(0, i);
        return ans;
    }
};

template <typename T>
struct dt<1, T> {
    static T det(const Matrix<1, 1, T>& m) { return m[0][0]; }
};

template <size_t row, size_t col, typename T>
class Matrix {
    Vector<col, T> raw[row];

   public:
    Matrix() : raw(){};
    Vector<col, T>& operator[](const size_t& i) {
        assert(i < row);
        return raw[i];
    }
    const Vector<col, T>& operator[](const size_t& i) const {
        assert(i < row);
        return raw[i];
    }
    Vector<row, T> column(const size_t& c) const {
        assert(c < col);
        Vector<row, T> ret;
        for (size_t i = 0; i < row; ++i) ret[i] = raw[i][c];
        return ret;
    }

    void set_column(const size_t& c, const Vector<row, T>& v) {
        assert(c < col);
        for (size_t i = 0; i < row; ++i) raw[i][c] = v[i];
    }

    static Matrix<row, col, T> identity() {
        Matrix<row, col, T> ret;
        for (size_t i = 0; i < row; ++i)
            for (size_t j = 0; j < col; ++j) ret[i][j] = static_cast<T>(i == j ? 1 : 0);
        return ret;
    }

    T det() const { return dt<col, T>::det(*this); }

    Matrix<row - 1, col - 1, T> get_minor(const size_t& t_row, const size_t& t_col) const {
        Matrix<row - 1, col - 1, T> ret;
        for (size_t i = 0; i < row - 1; ++i)
            for (size_t j = 0; j < col - 1; ++j) ret[i][j] = raw[i < t_row ? i : i + 1][j < t_col ? j : j + 1];
        return ret;
    }

    T cofactor(const size_t& t_row, const size_t& t_col) const {
        return get_minor(t_row, t_col).det() * ((t_row + t_col) % 2 ? -1 : 1);
    }
    Matrix<row, col, T> adjugate() const {
        Matrix<row, col, T> ret;
        for (size_t i = 0; i < row; ++i)
            for (size_t j = 0; j < col; ++j) ret[i][j] = cofactor(i, j);
        return ret;
    }

    Matrix<row, col, T> invert_transpose() {
        Matrix<row, col, T> ret = adjugate();
        T tmp = ret[0] * raw[0];
        return ret / tmp;
    }

    Matrix<col, row, T> transpose() {
        Matrix<col, row, T> ret;
        for (size_t i = 0; i < col; ++i)
            for (size_t j = 0; j < row; ++j) ret[i][j] = raw[j][i];
        return ret;
    }

    Matrix<row, col, T> invert() { return invert_transpose().transpose(); }
};

template <size_t row, size_t col, typename T>
Vector<row, T> operator*(const Matrix<row, col, T>& lhs, const Vector<col, T>& rhs) {
    Vector<row, T> ret;
    for (size_t i = 0; i < row; ++i) ret[i] = lhs[i] * rhs;
    return ret;
}

template <size_t row, size_t col, size_t col2, typename T>
Matrix<row, col2, T> operator*(const Matrix<row, col, T>& lhs, const Matrix<col, col2, T>& rhs) {
    Matrix<row, col2, T> ret;
    for (size_t i = 0; i < row; ++i)
        for (size_t j = 0; j < col2; ++j) ret[i][j] = lhs[i] * rhs.column(j);
    return ret;
}

template <size_t row, size_t col, typename T>
Matrix<row, col, T> operator/(Matrix<row, col, T> lhs, const T& rhs) {
    for (size_t i = 0; i < row; ++i) lhs[i] = lhs[i] / rhs;
    return lhs;
}

template <size_t row, size_t col, typename T>
std::ostream& operator<<(std::ostream& s, Matrix<row, col, T>& m) {
    s << "[" << std::endl;
    for (size_t i = 0; i < row; ++i) s << m[i] << std::endl;
    s << "]";
    return s;
}

using Vec2f = Vector<2, float>;
using Vec2i = Vector<2, int>;
using Vec3f = Vector<3, float>;
using Vec3i = Vector<3, int>;
using Vec4f = Vector<4, float>;
using Matrix4x4 = Matrix<4, 4, float>;
