using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEngine;

public class Tetrahedron
{
    public Vector3 p1;
    public Vector3 p2;
    public Vector3 p3;
    public Vector3 p4;
    
    public int i1;
    public int i2;
    public int i3;
    public int i4;

    public Vector3 normFace1;
    public Vector3 normFace2;
    public Vector3 normFace3;
    public Vector3 normFace4;
    
    public Vector3 midFacePos1;
    public Vector3 midFacePos2;
    public Vector3 midFacePos3;
    public Vector3 midFacePos4;



    public float mass;
    public float volume;

    public Tetrahedron(
        Vector3 p1, 
        Vector3 p2, 
        Vector3 p3, 
        Vector3 p4,
        int i1,
        int i2,
        int i3,
        int i4,
        float density)
    {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
        this.i1 = i1;
        this.i2 = i2;
        this.i3 = i3;
        this.i4 = i4;
        mass = CalculateMass(density);
        
        normFace1 = Vector3.Cross(p2 - p4, p3 - p4).normalized * 10;
        normFace2 = Vector3.Cross(p3 - p4, p1 - p4).normalized * 10;
        normFace3 = Vector3.Cross(p1 - p4, p2 - p4).normalized * 10;
        normFace4 = Vector3.Cross(p1 - p2, p3 - p2).normalized * 10;

        midFacePos1 = (p2 + p3 + p4) / 3;
        midFacePos2 = (p1 + p3 + p4) / 3;
        midFacePos3 = (p1 + p2 + p4) / 3;
        midFacePos4 = (p1 + p2 + p3) / 3;

    }

    private float CalculateMass(float density)
    {
        CalculateVolume();
        return density * volume;
    }

    private void CalculateVolume()
    { 
        volume = Mathf.Abs(Vector3.Dot(p2 - p1, Vector3.Cross(p3 - p1, p4 - p1)));
        volume /= 6;
    }

    public void RecalculateMass(float density)
    {
        CalculateVolume();
        mass = density * volume;
    }

}
