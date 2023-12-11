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
	Quaternion conjugate;

	conjugate.w = quaternion.w;
	conjugate.x = -quaternion.x;
	conjugate.y = -quaternion.y;
	conjugate.z = -quaternion.z;

	return conjugate;

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
	float norm = Norm(quaternion);

	if (norm != 0.0)
	{
		quaternion.w / norm;
		quaternion.x / norm;
		quaternion.y / norm;
		quaternion.z / norm;
	}

	return quaternion;
}




Quaternion InverseQuaternion(const Quaternion& quaternion)
{
	float normSquared = Norm(quaternion) * Norm(quaternion);

	// ノルムの2乗が0でない場合のみ逆元を計算
	if (normSquared != 0.0)
	{
		float reciprocalNormSquared = 1.0 /normSquared;

		return  Conjugate(quaternion) * reciprocalNormSquared;
	}

	return quaternion;

}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Quaternion q1 = { 2.0f,3.0f,4.0f,1.0f };
	Quaternion q2 = { 1.0f,3.0f,5.0f,2.0f };
	Quaternion identity = IdentityQuaternion();
	Quaternion conj = Conjugate(q1);
	Quaternion inv = InverseQuaternion(q1);
	Quaternion normal = NormalizeQuaternion(q1);
	Quaternion mul1= q1* q2;
	Quaternion mul2 = q2 * q1;
	float norm = Norm(q1);
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
		Novice::ScreenPrintf(0, 0, "Multiply: w=%.2f, x=%.2f, y=%.2f, z=%.2f\n", mul1.x, mul1.y, mul1.z, mul1.w);
		Novice::ScreenPrintf(0, 50, "Multiply: w=%.2f, x=%.2f, y=%.2f, z=%.2f\n", mul2.x, mul2.y, mul2.z, mul2.w);

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
