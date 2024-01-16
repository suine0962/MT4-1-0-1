#include <Novice.h>
#include"Matrix4x4.h"
#include"Vector3.h"
#include"Vector2.h"
#include<cmath>
#include <cassert>
#define _USE_MATH_DEFINES

static const int kRowHeight = 20;
static const int kColumnWidth = 60;
static const int kWindowWidth = 1280;
static const int kWindowHeght = 720;

const char kWindowTitle[] = "学籍番号";

struct Quaternion {
	float x;
	float y;
	float z;
	float w;
};

Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix)
{
	Vector3 result;

	result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + 1.0f * matrix.m[3][0];
	result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + 1.0f * matrix.m[3][1];
	result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + 1.0f * matrix.m[3][2];
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + 1.0f * matrix.m[3][3];

	assert(w != 0.0f);

	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;
}


Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {

	Matrix4x4 result;

	for (int y = 0; y < 4; y++) {
		for (int x = 0; x < 4; x++) {
			result.m[y][x] = m1.m[y][x] + m2.m[y][x];
		}
	}
	return result;
}

Vector3 Subtract(const Vector3& v1, const Vector3& v2)
{
	Vector3 result;
	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;
	return result;
}


Quaternion QuaternionSubtract(const Quaternion& q1, const Quaternion& q2)
{
	Quaternion result;
	result.x = q1.x - q2.x;
	result.y = q1.y - q2.y;
	result.z = q1.z - q2.z;
	result.w = q1.w - q2.w;
	return result;
}

float Dot(const Vector3& v1, const Vector3& v2)
{
	return  v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float Length(const Vector3& v)
{
	return sqrtf(Dot(v, v));
}

Vector3 Normalize(const Vector3& v)
{
	float length = Length(v);
	if (length != 0)
	{
		return{ v.x / length,v.y / length,v.z / length };
	}
	return v;
}

Vector3 Cross(const Vector3& v1, const Vector3& v2)
{
	Vector3 v = { v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x };

	return v;
};

Vector3 VectorMultiply(float sclar, const Vector3& v)
{
	return { sclar * v.x,sclar * v.y,sclar * v.z };
}

Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 result;

	result.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0] +
		m1.m[0][3] * m2.m[3][0];
	result.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1] +
		m1.m[0][3] * m2.m[3][1];
	result.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2] +
		m1.m[0][3] * m2.m[3][2];
	result.m[0][3] = m1.m[0][0] * m2.m[0][3] + m1.m[0][1] * m2.m[1][3] + m1.m[0][2] * m2.m[2][3] +
		m1.m[0][3] * m2.m[3][3];

	result.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0] +
		m1.m[1][3] * m2.m[3][0];
	result.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1] +
		m1.m[1][3] * m2.m[3][1];
	result.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2] +
		m1.m[1][3] * m2.m[3][2];
	result.m[1][3] = m1.m[1][0] * m2.m[0][3] + m1.m[1][1] * m2.m[1][3] + m1.m[1][2] * m2.m[2][3] +
		m1.m[1][3] * m2.m[3][3];

	result.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0] +
		m1.m[2][3] * m2.m[3][0];
	result.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1] +
		m1.m[2][3] * m2.m[3][1];
	result.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2] +
		m1.m[2][3] * m2.m[3][2];
	result.m[2][3] = m1.m[2][0] * m2.m[0][3] + m1.m[2][1] * m2.m[1][3] + m1.m[2][2] * m2.m[2][3] +
		m1.m[2][3] * m2.m[3][3];

	result.m[3][0] = m1.m[3][0] * m2.m[0][0] + m1.m[3][1] * m2.m[1][0] + m1.m[3][2] * m2.m[2][0] +
		m1.m[3][3] * m2.m[3][0];
	result.m[3][1] = m1.m[3][0] * m2.m[0][1] + m1.m[3][1] * m2.m[1][1] + m1.m[3][2] * m2.m[2][1] +
		m1.m[3][3] * m2.m[3][1];
	result.m[3][2] = m1.m[3][0] * m2.m[0][2] + m1.m[3][1] * m2.m[1][2] + m1.m[3][2] * m2.m[2][2] +
		m1.m[3][3] * m2.m[3][2];
	result.m[3][3] = m1.m[3][0] * m2.m[0][3] + m1.m[3][1] * m2.m[1][3] + m1.m[3][2] * m2.m[2][3] +
		m1.m[3][3] * m2.m[3][3];

	return result;
}


Matrix4x4 MakeIdenttity4x4()
{
	Matrix4x4 result;

	result.m[0][0] = 1;
	result.m[0][1] = 0;
	result.m[0][2] = 0;
	result.m[0][3] = 0;

	result.m[1][0] = 0;
	result.m[1][1] = 1;
	result.m[1][2] = 0;
	result.m[1][3] = 0;

	result.m[2][0] = 0;
	result.m[2][1] = 0;
	result.m[2][2] = 1;
	result.m[2][3] = 0;

	result.m[3][0] = 0;
	result.m[3][1] = 0;
	result.m[3][2] = 0;
	result.m[3][3] = 1;

	return result;
}

Matrix4x4 MakeRotateMatrix(const Vector3& rot) {

	Matrix4x4 rotate[3];

	// X回転
	rotate[0].m[0][0] = 1;
	rotate[0].m[0][1] = 0;
	rotate[0].m[0][2] = 0;
	rotate[0].m[0][3] = 0;

	rotate[0].m[1][0] = 0;
	rotate[0].m[1][1] = std::cos(rot.x);
	rotate[0].m[1][2] = std::sin(rot.x);
	rotate[0].m[1][3] = 0;

	rotate[0].m[2][0] = 0;
	rotate[0].m[2][1] = -std::sin(rot.x);
	rotate[0].m[2][2] = std::cos(rot.x);
	rotate[0].m[2][3] = 0;

	rotate[0].m[3][0] = 0;
	rotate[0].m[3][1] = 0;
	rotate[0].m[3][2] = 0;
	rotate[0].m[3][3] = 1;

	// Y回転
	rotate[1].m[0][0] = std::cos(rot.y);
	rotate[1].m[0][1] = 0;
	rotate[1].m[0][2] = -std::sin(rot.y);
	rotate[1].m[0][3] = 0;

	rotate[1].m[1][0] = 0;
	rotate[1].m[1][1] = 1;
	rotate[1].m[1][2] = 0;
	rotate[1].m[1][3] = 0;

	rotate[1].m[2][0] = std::sin(rot.y);
	rotate[1].m[2][1] = 0;
	rotate[1].m[2][2] = std::cos(rot.y);
	rotate[1].m[2][3] = 0;

	rotate[1].m[3][0] = 0;
	rotate[1].m[3][1] = 0;
	rotate[1].m[3][2] = 0;
	rotate[1].m[3][3] = 1;

	// Z回転
	rotate[2].m[0][0] = std::cos(rot.z);
	rotate[2].m[0][1] = std::sin(rot.z);
	rotate[2].m[0][2] = 0;
	rotate[2].m[0][3] = 0;

	rotate[2].m[1][0] = -std::sin(rot.z);
	rotate[2].m[1][1] = std::cos(rot.z);
	rotate[2].m[1][2] = 0;
	rotate[2].m[1][3] = 0;

	rotate[2].m[2][0] = 0;
	rotate[2].m[2][1] = 0;
	rotate[2].m[2][2] = 1;
	rotate[2].m[2][3] = 0;

	rotate[2].m[3][0] = 0;
	rotate[2].m[3][1] = 0;
	rotate[2].m[3][2] = 0;
	rotate[2].m[3][3] = 1;

	Matrix4x4 rotateMultiply = Multiply(rotate[0], Multiply(rotate[1], rotate[2]));

	return rotateMultiply;
}


Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rot, const Vector3& translate) {



	// スケーリング行列
	Matrix4x4 matScale{ 0 };
	matScale.m[0][0] = scale.x;
	matScale.m[1][1] = scale.y;
	matScale.m[2][2] = scale.z;

	matScale.m[3][3] = 1;

	// X軸回転まわりの回転

	Matrix4x4 matRotX = { 0 };

	matRotX.m[0][0] = 1;
	matRotX.m[1][1] = cosf(rot.x);
	matRotX.m[2][1] = -sinf(rot.x);
	matRotX.m[1][2] = sinf(rot.x);
	matRotX.m[2][2] = cosf(rot.x);
	matRotX.m[3][3] = 1;

	// Y軸まわりの回転
	Matrix4x4 matRotY = { 0 };
	matRotY.m[0][0] = cosf(rot.y);
	matRotY.m[1][1] = 1;
	matRotY.m[0][2] = -sinf(rot.y);
	matRotY.m[2][0] = sinf(rot.y);
	matRotY.m[2][2] = cosf(rot.y);
	matRotY.m[3][3] = 1;

	// Z軸まわりの回転
	Matrix4x4 matRotZ = { 0 };
	matRotZ.m[0][0] = cosf(rot.z);
	matRotZ.m[1][0] = sinf(rot.z);
	matRotZ.m[0][1] = -sinf(rot.z);
	matRotZ.m[1][1] = cosf(rot.z);
	matRotZ.m[2][2] = 1;
	matRotZ.m[3][3] = 1;

	Matrix4x4 matRot = Multiply(Multiply(matRotZ, matRotX), matRotY);

	Matrix4x4 matTrans = { 0 };

	matTrans.m[0][0] = 1;
	matTrans.m[1][1] = 1;
	matTrans.m[2][2] = 1;
	matTrans.m[3][3] = 1;
	matTrans.m[3][0] = translate.x;
	matTrans.m[3][1] = translate.y;
	matTrans.m[3][2] = translate.z;


	Matrix4x4 matWorld = Multiply(Multiply(matScale, matRot), matTrans);

	return matWorld;
}


void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label)
{
	for (int row = 0; row < 4; ++row)
	{
		for (int column = 0; column < 4; ++column)
		{
			Novice::ScreenPrintf(
				x + column * kColumnWidth, y + row * kRowHeight + 20, "%6.02f", matrix.m[row][column]);
		}
	}
	Novice::ScreenPrintf(x, y, "%s", label);
}

Matrix4x4 Inverse(const Matrix4x4& m)
{
	Matrix4x4 result;

	float date = m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3] +
		m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1] +
		m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2] -

		m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1] -
		m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3] -
		m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2] -

		m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3] -
		m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1] -
		m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2] +

		m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1] +
		m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3] +
		m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2] +

		m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3] +
		m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1] +
		m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2] -

		m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1] -
		m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3] -
		m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2] -

		m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0] -
		m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0] -
		m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0] +

		m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0] +
		m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0] +
		m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0];

	float dateRect = -sqrtf(date * date);

	result.m[0][0] = (
		m.m[1][1] * m.m[2][2] * m.m[3][3] +
		m.m[1][2] * m.m[2][3] * m.m[3][1] +
		m.m[1][3] * m.m[2][1] * m.m[3][2] -
		m.m[1][3] * m.m[2][2] * m.m[3][1] -
		m.m[1][2] * m.m[2][1] * m.m[3][3] -
		m.m[1][1] * m.m[2][3] * m.m[3][2])
		/ dateRect;

	result.m[0][1] = (
		-m.m[0][1] * m.m[2][2] * m.m[3][3] -
		m.m[0][2] * m.m[2][3] * m.m[3][1] -
		m.m[0][3] * m.m[2][1] * m.m[3][2] +
		m.m[0][3] * m.m[2][2] * m.m[3][1] +
		m.m[0][2] * m.m[2][1] * m.m[3][3] +
		m.m[0][1] * m.m[2][3] * m.m[3][2])
		/ dateRect;

	result.m[0][2] = (
		m.m[0][1] * m.m[1][2] * m.m[3][3] +
		m.m[0][2] * m.m[1][3] * m.m[3][1] +
		m.m[0][3] * m.m[1][1] * m.m[3][2] -
		m.m[0][3] * m.m[1][2] * m.m[3][1] -
		m.m[0][2] * m.m[1][1] * m.m[3][3] -
		m.m[0][1] * m.m[1][3] * m.m[3][2])
		/ dateRect;

	result.m[0][3] = (
		-m.m[0][1] * m.m[1][2] * m.m[2][3] -
		m.m[0][2] * m.m[1][3] * m.m[2][1] -
		m.m[0][3] * m.m[1][1] * m.m[2][2] +
		m.m[0][3] * m.m[1][2] * m.m[2][1] +
		m.m[0][2] * m.m[1][1] * m.m[2][3] +
		m.m[0][1] * m.m[1][3] * m.m[2][2])
		/ dateRect;

	result.m[1][0] = (
		-m.m[1][0] * m.m[2][2] * m.m[3][3] -
		m.m[1][2] * m.m[2][3] * m.m[3][0] -
		m.m[1][3] * m.m[2][0] * m.m[3][2] +
		m.m[1][3] * m.m[2][2] * m.m[3][0] +
		m.m[1][2] * m.m[2][0] * m.m[3][3] +
		m.m[1][0] * m.m[2][3] * m.m[3][2])
		/ dateRect;

	result.m[1][1] = (
		m.m[0][0] * m.m[2][2] * m.m[3][3] +
		m.m[0][2] * m.m[2][3] * m.m[3][0] +
		m.m[0][3] * m.m[2][0] * m.m[3][2] -
		m.m[0][3] * m.m[2][2] * m.m[3][0] -
		m.m[0][2] * m.m[2][0] * m.m[3][3] -
		m.m[0][0] * m.m[2][3] * m.m[3][2])
		/ dateRect;

	result.m[1][2] = (
		-m.m[0][0] * m.m[1][2] * m.m[3][3] -
		m.m[0][2] * m.m[1][3] * m.m[3][0] -
		m.m[0][3] * m.m[1][0] * m.m[3][2] +
		m.m[0][3] * m.m[1][2] * m.m[3][0] +
		m.m[0][2] * m.m[1][0] * m.m[3][3] +
		m.m[0][0] * m.m[1][3] * m.m[3][2])
		/ dateRect;

	result.m[1][3] = (
		m.m[0][0] * m.m[1][2] * m.m[2][3] +
		m.m[0][2] * m.m[1][3] * m.m[2][0] +
		m.m[0][3] * m.m[1][0] * m.m[2][2] -
		m.m[0][3] * m.m[1][2] * m.m[2][0] -
		m.m[0][2] * m.m[1][0] * m.m[2][3] -
		m.m[0][0] * m.m[1][3] * m.m[2][2])
		/ dateRect;

	result.m[2][0] = (
		m.m[1][0] * m.m[2][1] * m.m[3][3] +
		m.m[1][1] * m.m[2][3] * m.m[3][0] +
		m.m[1][3] * m.m[2][0] * m.m[3][1] -
		m.m[1][3] * m.m[2][1] * m.m[3][0] -
		m.m[1][1] * m.m[2][0] * m.m[3][3] -
		m.m[1][0] * m.m[2][3] * m.m[3][1])
		/ dateRect;

	result.m[2][1] = (
		-m.m[0][0] * m.m[2][1] * m.m[3][3] -
		m.m[0][1] * m.m[2][3] * m.m[3][0] -
		m.m[0][3] * m.m[2][0] * m.m[3][1] +
		m.m[0][3] * m.m[2][1] * m.m[3][0] +
		m.m[0][1] * m.m[2][0] * m.m[3][3] +
		m.m[0][0] * m.m[2][3] * m.m[3][1])
		/ dateRect;

	result.m[2][2] = (
		m.m[0][0] * m.m[1][1] * m.m[3][3] +
		m.m[0][1] * m.m[1][3] * m.m[3][0] +
		m.m[0][3] * m.m[1][0] * m.m[3][1] -
		m.m[0][3] * m.m[1][1] * m.m[3][0] -
		m.m[0][1] * m.m[1][0] * m.m[3][3] -
		m.m[0][0] * m.m[1][3] * m.m[3][1])
		/ dateRect;

	result.m[2][3] = (
		-m.m[0][0] * m.m[1][1] * m.m[2][3] -
		m.m[0][1] * m.m[1][3] * m.m[2][0] -
		m.m[0][3] * m.m[1][0] * m.m[2][1] +
		m.m[0][3] * m.m[1][1] * m.m[2][0] +
		m.m[0][1] * m.m[1][0] * m.m[2][3] +
		m.m[0][0] * m.m[1][3] * m.m[2][1])
		/ dateRect;

	result.m[3][0] = (
		-m.m[1][0] * m.m[2][1] * m.m[3][2] -
		m.m[1][1] * m.m[2][2] * m.m[3][0] -
		m.m[1][2] * m.m[2][0] * m.m[3][1] +
		m.m[1][2] * m.m[2][1] * m.m[3][0] +
		m.m[1][1] * m.m[2][0] * m.m[3][2] +
		m.m[1][0] * m.m[2][2] * m.m[3][1])
		/ dateRect;

	result.m[3][1] = (
		m.m[0][0] * m.m[2][1] * m.m[3][2] +
		m.m[0][1] * m.m[2][2] * m.m[3][0] +
		m.m[0][2] * m.m[2][0] * m.m[3][1] -
		m.m[0][2] * m.m[2][1] * m.m[3][0] -
		m.m[0][1] * m.m[2][0] * m.m[3][2] -
		m.m[0][0] * m.m[2][2] * m.m[3][1])
		/ dateRect;

	result.m[3][2] = (
		-m.m[0][0] * m.m[1][1] * m.m[3][2] -
		m.m[0][1] * m.m[1][2] * m.m[3][0] -
		m.m[0][2] * m.m[1][0] * m.m[3][1] +
		m.m[0][2] * m.m[1][1] * m.m[3][0] +
		m.m[0][1] * m.m[1][0] * m.m[3][2] +
		m.m[0][0] * m.m[1][2] * m.m[3][1])
		/ dateRect;

	result.m[3][3] = (
		m.m[0][0] * m.m[1][1] * m.m[2][2] +
		m.m[0][1] * m.m[1][2] * m.m[2][0] +
		m.m[0][2] * m.m[1][0] * m.m[2][1] -
		m.m[0][2] * m.m[1][1] * m.m[2][0] -
		m.m[0][1] * m.m[1][0] * m.m[2][2] -
		m.m[0][0] * m.m[1][2] * m.m[2][1])
		/ dateRect;

	return result;
}


/// 透視投影行列
	/// </summary>
	/// <param name="fovY"></param>
	/// <param name="aspectRatio"></param>
	/// <param name="nearClip"></param>
	/// <param name="farClip"></param>
	/// <returns></returns>
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result;
	float tangent = tan(fovY / 2);
	float cot = 1 / tangent;


	result.m[0][0] = 1 / aspectRatio * cot;
	result.m[0][1] = 0;
	result.m[0][2] = 0;
	result.m[0][3] = 0;

	result.m[1][0] = 0;
	result.m[1][1] = cot;
	result.m[1][2] = 0;
	result.m[1][3] = 0;

	result.m[2][0] = 0;
	result.m[2][1] = 0;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1;

	result.m[3][0] = 0;
	result.m[3][1] = 0;
	result.m[3][2] = -nearClip * farClip / (farClip - nearClip);
	result.m[3][3] = 0;

	return result;
}

Matrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip) {
	Matrix4x4 result;

	result.m[0][0] = 2 / (right - left);
	result.m[0][1] = 0;
	result.m[0][2] = 0;
	result.m[0][3] = 0;

	result.m[1][0] = 0;
	result.m[1][1] = 2 / (top - bottom);
	result.m[1][2] = 0;
	result.m[1][3] = 0;

	result.m[2][0] = 0;
	result.m[2][1] = 0;
	result.m[2][2] = 1 / (farClip - nearClip);
	result.m[2][3] = 0;

	result.m[3][0] = (left + right) / (left - right);
	result.m[3][1] = (top + bottom) / (bottom - top);
	result.m[3][2] = nearClip / (nearClip - farClip);
	result.m[3][3] = 1;

	return result;

}

Matrix4x4 ScalarMultiply(float scalar, const Matrix4x4& m)
{

	Matrix4x4 result;

	for (int y = 0; y < 4; y++) {
		for (int x = 0; x < 4; x++) {
			result.m[y][x] = scalar * m.m[y][x];
		}
	}
	return result;
}


Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float angel)
{
	Matrix4x4 rS = MakeIdenttity4x4();

	rS.m[0][0] = std::cosf(angel);
	rS.m[1][1] = std::cosf(angel);
	rS.m[2][2] = std::cosf(angel);

	Matrix4x4 rP = MakeIdenttity4x4();

	rP.m[0][0] = axis.x * axis.x;
	rP.m[0][1] = axis.x * axis.y;
	rP.m[0][2] = axis.x * axis.z;

	rP.m[1][0] = axis.y * axis.x;
	rP.m[1][1] = axis.y * axis.y;
	rP.m[1][2] = axis.y * axis.z;

	rP.m[2][0] = axis.z * axis.x;
	rP.m[2][1] = axis.z * axis.y;
	rP.m[2][2] = axis.z * axis.z;

	rP.m[3][3] = 0.0f;

	rP = ScalarMultiply((1.0f - std::cosf(angel)), rP);

	Matrix4x4 rC = MakeIdenttity4x4();

	rC.m[0][0] = 0.0f;
	rC.m[0][1] = -axis.z;
	rC.m[0][2] = axis.y;
	rC.m[1][0] = axis.z;
	rC.m[1][1] = 0.0f;
	rC.m[1][2] = -axis.x;
	rC.m[2][0] = -axis.y;
	rC.m[2][1] = axis.x;
	rC.m[2][2] = 0.0f;
	rC.m[3][3] = 0.0f;

	rC = ScalarMultiply((-std::sinf(angel)), rC);

	Matrix4x4 resultMatrix = Add(Add(rS, rP), rC);

	return resultMatrix;
}


Matrix4x4 DerectionToDerection(const Vector3& from, const Vector3& to)
{
	Vector3 Normal = Normalize(Cross(from, to));
	Vector3 MathTo = VectorMultiply(-1.0f, to);
	Matrix4x4 Result = MakeIdenttity4x4();

	if ((from.x == MathTo.x &&
		from.y == MathTo.y &&
		from.z == MathTo.z)) {
		if (from.x != 0.0f || from.y != 0.0f) {
			Normal = { from.y, -from.x, 0.0f };
		}
		else if (from.x != 0.0f || from.z != 0.0f) {
			Normal = { from.z, 0.0f, -from.x };
		}
	}
	float cos = Dot(from, to);
	float sin = Length(Cross(from, to));

	Result.m[0][0] = Normal.x * Normal.x * (1.0f - cos) + cos;
	Result.m[0][1] = Normal.x * Normal.y * (1.0f - cos) + Normal.z * sin;
	Result.m[0][2] = Normal.x * Normal.z * (1.0f - cos) - Normal.y * sin;

	Result.m[1][0] = Normal.x * Normal.y * (1.0f - cos) - Normal.z * sin;
	Result.m[1][1] = Normal.y * Normal.y * (1.0f - cos) + cos;
	Result.m[1][2] = Normal.y * Normal.z * (1.0f - cos) + Normal.x * sin;

	Result.m[2][0] = Normal.x * Normal.z * (1.0f - cos) + Normal.y * sin;
	Result.m[2][1] = Normal.y * Normal.z * (1.0f - cos) - Normal.x * sin;
	Result.m[2][2] = Normal.z * Normal.z * (1.0f - cos) + cos;

	return Result;
}

Quaternion operator*(const Quaternion& a, const float b) {
	return Quaternion{ .x = a.x * b,.y = a.y * b,.z = a.z * b,.w = a.w * b, };
}

Quaternion operator*(const float a,const Quaternion& b) {
	return b * a;
}

//Multiply
Quaternion operator*(const Quaternion& lns, const Quaternion& rhs)
{
	Quaternion q;

	q.w = lns.w * rhs.w - lns.x * rhs.x -
		lns.y * rhs.y - lns.z * rhs.z;

	q.x = lns.y * rhs.z - lns.z * rhs.y +
		rhs.w * lns.x + lns.w * rhs.x;

	q.y = lns.z * rhs.x - lns.x * rhs.z +
		rhs.w * lns.y + lns.w * rhs.y;

	q.z = lns.x * rhs.y - lns.y * rhs.x +
		rhs.w * lns.z + lns.w * rhs.z;

	return q;

}


Quaternion QuaternionMultiply(const Quaternion& lns, const Quaternion& rhs)
{
	Quaternion q;

	q.w = lns.w * rhs.w - lns.x * rhs.x -
		lns.y * rhs.y - lns.z * rhs.z;

	q.x = lns.y * rhs.z - lns.z * rhs.y +
		rhs.w * lns.x + lns.w * rhs.x;

	q.y = lns.z * rhs.x - lns.x * rhs.z +
		rhs.w * lns.y + lns.w * rhs.y;

	q.z = lns.x * rhs.y - lns.y * rhs.x +
		rhs.w * lns.z + lns.w * rhs.z;

	return q;

}
Quaternion IdentityQuaternion()
{
	Quaternion identity;

	identity.w = 1.0f;
	identity.x = 0.0f;
	identity.y = 0.0f;
	identity.z = 0.0f;

	return identity;
}


Quaternion Conjugate(const Quaternion& quaternion)
{
	Quaternion result =
	{
		-quaternion.x,
		-quaternion.y,
		-quaternion.z,
		quaternion.w
	};

	return result;

}

Quaternion SlerpConjugate(const Quaternion& quaternion)
{
	Quaternion result =
	{
		quaternion.x,
		-quaternion.y,
		quaternion.z,
		-quaternion.w
	};

	return result;

}

float Norm(const Quaternion& quaternion)
{
	return sqrt(quaternion.w * quaternion.w
		+ quaternion.x * quaternion.x
		+ quaternion.y * quaternion.y
		+ quaternion.z * quaternion.z);
}


Quaternion NormalizeQuaternion(const Quaternion& quaternion)
{
	Quaternion result = {};

	float norm = Norm(quaternion);

	if (quaternion.x != 0.0f)
	{
		result.x = quaternion.x / norm;
	}else
	{
		result.x = 0.0f;
	}

	if (quaternion.y != 0.0f)
	{
		result.y = quaternion.y / norm;
	}else
	{
		result.y = 0.0f;
	}
	


	if (quaternion.z != 0.0f)
	{
		result.z = quaternion.z / norm;
	}else
	{
		result.z = 0.0f;
	}

	if (quaternion.w != 0.0f)
	{
		result.w = quaternion.w / norm;
	}else
	{
		result.w = 0.0f;
	}

	return result;
}

void VectorScreenPrintf(int x, int y, Vector3& vector, const char* label)
{
	Novice::ScreenPrintf(x, y, "%0.2f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%0.2f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%0.2f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);
}


Quaternion InverseQuaternion(const Quaternion& quaternion)
{
	
	Quaternion result = {};
	Quaternion conjugate = Conjugate(quaternion);

	float norm = Norm(quaternion);

	if (norm != 0.0f) {
		result.x = conjugate.x / (norm * norm);
		result.y = conjugate.y / (norm * norm);
		result.z = conjugate.z / (norm * norm);
		result.w = conjugate.w / (norm * norm);
	}

	return result;
}

//任意軸回転
Quaternion MakeRotateAxisAngleQuaternion(
	const Vector3& axis, float angl)
{
	Quaternion result;
	float quaternionSin = std::sinf(angl / 2.0f);

	result.w = std::cosf(angl / 2.0f);
	result.x = quaternionSin * axis.x;
	result.y = quaternionSin * axis.y;
	result.z = quaternionSin * axis.z;

	return result;
}


//ベクトルでquaternionを回転させた結果のベクトルを求める
Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion)
{
	Quaternion vectorQuat = { 0, vector.x, vector.y, vector.z };

	// 回転
	Quaternion rotatedQuat = QuaternionMultiply(QuaternionMultiply(quaternion, vectorQuat), Conjugate(quaternion));

	// 結果をベクトルに変換
	Vector3 rotatedVector = { rotatedQuat.x, rotatedQuat.y, rotatedQuat.z };

	return rotatedVector;

}

//Quaternionから回転行列を求める
Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion)
{
	Matrix4x4 result;

	result.m[0][0] = (quaternion.w * quaternion.w) + (quaternion.x * quaternion.x) -
		(quaternion.y * quaternion.y) - (quaternion.z * quaternion.z);
	result.m[0][1] = (quaternion.x * quaternion.y) + (quaternion.w * quaternion.z) * 2;
	result.m[0][2] = (quaternion.x * quaternion.z) - (quaternion.w * quaternion.y) * 2;
	result.m[0][3] = 0.0f;

	result.m[1][0] = (quaternion.x * quaternion.y) - (quaternion.w * quaternion.z) * 2;
	result.m[1][1] = (quaternion.w * quaternion.w) - (quaternion.x * quaternion.x) +
		(quaternion.y * quaternion.y) - (quaternion.z * quaternion.z);
	result.m[1][2] = (quaternion.y * quaternion.z) + (quaternion.w * quaternion.x) * 2;
	result.m[1][3] = 0.0f;

	result.m[2][0] = (quaternion.x * quaternion.z) + (quaternion.w * quaternion.y) * 2;
	result.m[2][1] = (quaternion.y * quaternion.z) - (quaternion.w * quaternion.x) * 2;
	result.m[2][2] = (quaternion.w * quaternion.w) - (quaternion.x * quaternion.x) -
		(quaternion.y * quaternion.y) + (quaternion.z * quaternion.z);
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;
}

//球面線形保管

Quaternion Slerp(const Quaternion& q0, const Quaternion& q1,float t) 
{

	float dot = (q0.x * q1.x) + (q0.y * q1.y) + (q0.z * q1.z) + (q0.w * q1.w);//内積
	float theta = std::acos(dot);//θ

	if (dot < 0)
	{
		SlerpConjugate(q0);

		dot = -dot;
	}


	float scale0 = std::sin(theta) / std::sin((1 - t) * theta);
	float scale1 = std::sin(theta) / std::sin(t * theta);

	return  scale0 * q0 + scale1 * q1;

}


// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Quaternion rotation = MakeRotateAxisAngleQuaternion(
		Normalize(Vector3{ 1.0f,0.4f,-0.2f }), 0.45f);
	Vector3 pointY = { 2.1f,-0.9f,1.3f };
	Matrix4x4 rotateMatrix = MakeRotateMatrix(rotation);
	Vector3 rotateByQuaternion = RotateVector(pointY, rotation);
	Vector3 rotateByMatrix = Transform(pointY, rotateMatrix);

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		/// 
		/// 
		Novice::ScreenPrintf(20, 0, "%.2f,%.2f,%.2f,%.2f :rotation", rotation);
		MatrixScreenPrintf(0, kRowHeight * 1, rotateMatrix, "rotateMatrix");
		VectorScreenPrintf(0, kRowHeight * 6, rotateByQuaternion, "rotateByQuaternion");
		VectorScreenPrintf(0, kRowHeight * 7, rotateByMatrix, "rotateByMatrix");
		
		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
