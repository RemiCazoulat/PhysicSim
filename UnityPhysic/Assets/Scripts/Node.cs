using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    public Vector3 pos;
    public Vector3 vel;
    public Vector3 force;
    public float mass;
    public bool isFixed;
  
    public Node(Vector3 p, float mass)
    {
        pos = p;
        vel = new Vector3(0f, 0f, 0f);
        force =  new Vector3(0f, 0f, 0f);
        this.mass = mass;
        isFixed = false;
    }
    public Node(Vector3 p)
    {
        pos = p;
        vel = new Vector3(0f, 0f, 0f);
        force =  new Vector3(0f, 0f, 0f);
        mass = 0;
        isFixed = false;
    }
    
    public void ComputeGravity(Vector3 gravity)
    {
        force += mass * gravity;
    }
   

    public void Friction(float friction)
    {
        force -= friction * mass * vel;
    }
}
