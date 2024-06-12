using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Matrix3X3
{
    private float a00;
    private float a01;
    private float a02;
    private float a10;
    private float a11;
    private float a12;
    private float a20;
    private float a21;
    private float a22;

    public Matrix3X3(Vector3 verticalVector, Vector3 horizontalVector)
    {
        a00 = verticalVector.x * horizontalVector.x;
        a01 = verticalVector.x * horizontalVector.y;
        a02 = verticalVector.x * horizontalVector.z;
        a10 = verticalVector.y * horizontalVector.x;
        a11 = verticalVector.y * horizontalVector.y;
        a12 = verticalVector.y * horizontalVector.z;
        a20 = verticalVector.z * horizontalVector.x;
        a21 = verticalVector.z * horizontalVector.y;
        a22 = verticalVector.z * horizontalVector.z;

    }
    public Matrix3X3(Vector3 line1, Vector3 line2, Vector3 line3)
    {
        a00 = line1.x;
        a01 = line1.y;
        a02 = line1.z;
        a10 = line2.x;
        a11 = line2.y;
        a12 = line2.z;
        a20 = line3.x;
        a21 = line3.y;
        a22 = line3.z;

    }
    private Matrix3X3(
        float e00,
        float e01,
        float e02,
        float e10,
        float e11,
        float e12,
        float e20,
        float e21,
        float e22
        )
    {
        a00 =  e00;
        a01 =  e01;
        a02 =  e02;
        a10 =  e10;
        a11 =  e11;
        a12 =  e12;
        a20 =  e20;
        a21 =  e21;
        a22 =  e22;
    }
    
    

    public void AddFloat(float a)
    {
        a00 += a;
        a01 += a;
        a02 += a;
        a10 += a;
        a11 += a;
        a12 += a;
        a20 += a;
        a21 += a;
        a22 += a;
    }
    
    public void MultFloat(float a)
    {
        a00 *= a;
        a01 *= a;
        a02 *= a;
        a10 *= a;
        a11 *= a;
        a12 *= a;
        a20 *= a;
        a21 *= a;
        a22 *= a;
    }

    private Matrix3X3 Transpose()
    {
        return new Matrix3X3(
            a00, a10, a20,
            a01, a11, a21,
            a02, a12, a22
        );
    }

    private float Determinant3X3()
    {
        return       a00 * (a11 * a22 - a12 * a21) -
                     a01 * (a10 * a22 - a12 * a20) +
                     a02 * (a10 * a21 - a11 * a20);
    }
    
    private static float Determinant2X2(float a00, float a01, float a10, float a11)
    {
        return a00 * a11 - a01 * a10;
    }
    public Matrix3X3 Inverse()
    {
        var det = Determinant3X3();
        if (det == 0)
        {
            return Zero();
        }

        var transposed = Transpose();
        var adj00 =  Determinant2X2(transposed.a11, transposed.a12, transposed.a21, transposed.a22);
        var adj01 = -Determinant2X2(transposed.a10, transposed.a12, transposed.a20, transposed.a22);
        var adj02 =  Determinant2X2(transposed.a10, transposed.a11, transposed.a20, transposed.a21);
        var adj10 = -Determinant2X2(transposed.a01, transposed.a02, transposed.a21, transposed.a22);
        var adj11 =  Determinant2X2(transposed.a00, transposed.a02, transposed.a20, transposed.a22);
        var adj12 = -Determinant2X2(transposed.a00, transposed.a01, transposed.a20, transposed.a21);
        var adj20 =  Determinant2X2(transposed.a01, transposed.a02, transposed.a11, transposed.a12);
        var adj21 = -Determinant2X2(transposed.a00, transposed.a02, transposed.a10, transposed.a12);
        var adj22 =  Determinant2X2(transposed.a00, transposed.a01, transposed.a10, transposed.a11);

        Matrix3X3 adj = new Matrix3X3(
            adj00,
            adj01,
            adj02,
            adj10,
            adj11,
            adj12,
            adj20,
            adj21,
            adj22
        );
        adj.MultFloat(1 / det);
        return adj;
    }
    
    public static Matrix3X3 operator -(Matrix3X3 m1, Matrix3X3 m2)
    {
        return new Matrix3X3(
            m1.a00 - m2.a00,
            m1.a01 - m2.a01,
            m1.a02 - m2.a02,
            m1.a10 - m2.a10,
            m1.a11 - m2.a11,
            m1.a12 - m2.a12,
            m1.a20 - m2.a20,
            m1.a21 - m2.a21,
            m1.a22 - m2.a22
        );
    }
    public static Matrix3X3 operator +(Matrix3X3 m1, Matrix3X3 m2)
    {
        return new Matrix3X3(
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00,
            m1.a00 + m2.a00
        );
    }
    
    public static Matrix3X3 operator *(Matrix3X3 m1, Matrix3X3 m2)
    {
        var e00 = m1.a00 * m2.a00 + m1.a01 * m2.a10 + m1.a02 * m2.a20;
        var e01 = m1.a00 * m2.a01 + m1.a01 * m2.a11 + m1.a02 * m2.a21;
        var e02 = m1.a00 * m2.a02 + m1.a01 * m2.a12 + m1.a02 * m2.a22;
        var e10 = m1.a10 * m2.a00 + m1.a11 * m2.a10 + m1.a12 * m2.a20;
        var e11 = m1.a10 * m2.a01 + m1.a11 * m2.a11 + m1.a12 * m2.a21;
        var e12 = m1.a10 * m2.a02 + m1.a11 * m2.a12 + m1.a12 * m2.a22;
        var e20 = m1.a20 * m2.a00 + m1.a21 * m2.a10 + m1.a22 * m2.a20;
        var e21 = m1.a20 * m2.a01 + m1.a21 * m2.a11 + m1.a22 * m2.a21;
        var e22 = m1.a20 * m2.a02 + m1.a21 * m2.a12 + m1.a22 * m2.a22;

        return new Matrix3X3( e00, e01, e02, e10, e11, e12, e20, e21, e22);
    }

    public static Vector3 MatVecMultiply(Matrix3X3 mat, Vector3 vec)
    {
       var x = mat.a00 * vec.x + mat.a01 * vec.y + mat.a02 * vec.z;
       var y = mat.a10 * vec.x + mat.a11 * vec.y + mat.a12 * vec.z;
       var z = mat.a20 * vec.x + mat.a21 * vec.y + mat.a22 * vec.z;
        return new Vector3(x, y, z);
    }

    public static Matrix3X3 Zero()
    {
        return new Matrix3X3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
    public static Matrix3X3 Id()
    {
        return new Matrix3X3(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    public override string ToString()
    {
        string result = a00 + " - " + a01 + " - " + a02 + "\n" +
                        a10 + " - " + a11 + " - " + a12 + "\n" +
                        a20 + " - " + a21 + " - " + a22 + "\n";
        return result;
    }
}