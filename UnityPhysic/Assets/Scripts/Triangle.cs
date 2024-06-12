using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Triangle
{
    public int i1;
    public int i2;
    public int i3;
    public string ID;

    public Triangle(int i1, int i2, int i3)
    {
        this.i1 = i1;
        this.i2 = i2;
        this.i3 = i3;
        ID = GetTriangleID();
    }
    public string GetTriangleID()
    {
        string result = "";

        int min1 = Mathf.Min(i1, Mathf.Min(i2, i3));
        result += min1.ToString();
        if (min1 == i1) result += i2 < i3 ? i2.ToString() + i3 : i3.ToString() + i2;
        if (min1 == i2) result += i1 < i3 ? i1.ToString() + i3 : i3.ToString() + i1;
        if (min1 == i3) result += i1 < i2 ? i1.ToString() + i2 : i2.ToString() + i1;

        return result;
    }
}
