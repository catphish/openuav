// Copyright (C) 2022 Martin Weigel <mail@MartinWeigel.com>
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

/**
 * @file    Quaternion.c
 * @brief   A basic quaternion library written in C
 * @date    2022-05-16
 */
#include "quaternion.h"
#include <math.h>

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

void Quaternion_set(float w, float v1, float v2, float v3, Quaternion* output)
{
    output->w = w;
    output->v[0] = v1;
    output->v[1] = v2;
    output->v[2] = v3;
}

void Quaternion_setIdentity(Quaternion* q)
{
    Quaternion_set(1, 0, 0, 0, q);
}

void Quaternion_fromAxisAngle(float axis[3], float angle, Quaternion* output)
{
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
    output->w = cosf(angle / 2.f);
    float c = sinf(angle / 2.f);
    output->v[0] = c * axis[0];
    output->v[1] = c * axis[1];
    output->v[2] = c * axis[2];
}

float Quaternion_toAxisAngle(Quaternion* q, float output[3])
{
    // Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
    float angle = 2.f * acosf(q->w);
    float divider = sqrtf(1.f - q->w * q->w);

    if(divider != 0.f) {
        // Calculate the axis
        output[0] = q->v[0] / divider;
        output[1] = q->v[1] / divider;
        output[2] = q->v[2] / divider;
    } else {
        // Arbitrary normalized axis
        output[0] = 1;
        output[1] = 0;
        output[2] = 0;
    }
    return angle;
}

void Quaternion_fromXRotation(float angle, Quaternion* output)
{
    float axis[3] = {1.f, 0, 0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromYRotation(float angle, Quaternion* output)
{
    float axis[3] = {0, 1.f, 0};
    Quaternion_fromAxisAngle(axis, angle, output);
}

void Quaternion_fromZRotation(float angle, Quaternion* output)
{
    float axis[3] = {0, 0, 1.f};
    Quaternion_fromAxisAngle(axis, angle, output);
}

float Quaternion_norm(Quaternion* q)
{
    return sqrtf(q->w*q->w + q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2]);
}

void Quaternion_normalize(Quaternion* q, Quaternion* output)
{
    float len = Quaternion_norm(q);
    Quaternion_set(
        q->w / len,
        q->v[0] / len,
        q->v[1] / len,
        q->v[2] / len,
        output);
}

void Quaternion_multiply(Quaternion* q1, Quaternion* q2, Quaternion* output)
{
    Quaternion result;

    /*
    Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
             a*e - b*f - c*g - d*h
        + i (b*e + a*f + c*h- d*g)
        + j (a*g - b*h + c*e + d*f)
        + k (a*h + b*g - c*f + d*e)
    */
    result.w =    q1->w   *q2->w    - q1->v[0]*q2->v[0] - q1->v[1]*q2->v[1] - q1->v[2]*q2->v[2];
    result.v[0] = q1->v[0]*q2->w    + q1->w   *q2->v[0] + q1->v[1]*q2->v[2] - q1->v[2]*q2->v[1];
    result.v[1] = q1->w   *q2->v[1] - q1->v[0]*q2->v[2] + q1->v[1]*q2->w    + q1->v[2]*q2->v[0];
    result.v[2] = q1->w   *q2->v[2] + q1->v[0]*q2->v[1] - q1->v[1]*q2->v[0] + q1->v[2]*q2->w   ;

    *output = result;
}

void Quaternion_rotate(Quaternion* q, float v[3], float output[3])
{
    float result[3];

    float ww = q->w * q->w;
    float xx = q->v[0] * q->v[0];
    float yy = q->v[1] * q->v[1];
    float zz = q->v[2] * q->v[2];
    float wx = q->w * q->v[0];
    float wy = q->w * q->v[1];
    float wz = q->w * q->v[2];
    float xy = q->v[0] * q->v[1];
    float xz = q->v[0] * q->v[2];
    float yz = q->v[1] * q->v[2];

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
    // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
    // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

    result[0] = ww*v[0] + 2*wy*v[2] - 2*wz*v[1] +
                xx*v[0] + 2*xy*v[1] + 2*xz*v[2] -
                zz*v[0] - yy*v[0];
    result[1] = 2*xy*v[0] + yy*v[1] + 2*yz*v[2] +
                2*wz*v[0] - zz*v[1] + ww*v[1] -
                2*wx*v[2] - xx*v[1];
    result[2] = 2*xz*v[0] + 2*yz*v[1] + zz*v[2] -
                2*wy*v[0] - yy*v[2] + 2*wx*v[1] -
                xx*v[2] + ww*v[2];

    // Copy result to output
    output[0] = result[0];
    output[1] = result[1];
    output[2] = result[2];
}

// Calculate the dot product of two 3D vectors
void Vector_dot(float v1[3], float v2[3], float* output)
{
    *output = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

// Calculate the cross product of two 3D vectors
void Vector_cross(float v1[3], float v2[3], float output[3])
{
    output[0] = v1[1]*v2[2] - v1[2]*v2[1];
    output[1] = v1[2]*v2[0] - v1[0]*v2[2];
    output[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

#define ONE_MINUS_EPS (1.f - QUATERNION_EPS)

void Quaternion_from_unit_vecs(float v0[3], float v1[3], Quaternion* output)
{
    float dot;
    Vector_dot(v0, v1, &dot);

    if(dot > ONE_MINUS_EPS) {
        Quaternion_setIdentity(output);
        return;
    } else if(dot < -ONE_MINUS_EPS) {
        // Rotate along any orthonormal vec to vec1 or vec2 as the axis.
        float cross[3];
        Vector_cross((float[]){1.f,0,0}, v0, cross);
        Quaternion_fromAxisAngle(cross, M_PI, output);
        return;
    }

    float w = 1. + dot;
    float v[3];
    Vector_cross(v0, v1, v);

    Quaternion_set(w, v[0], v[1], v[2], output);
    Quaternion_normalize(output, output);
}