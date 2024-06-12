using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spring
{
    public readonly Node nodeA, nodeB;

    private float length0;
    private float length;
    private float stiffness;
    private float friction;
    public  float volume;


    public Spring(Node na, Node nb,float s, float friction)
    {
        nodeA = na;
        nodeB = nb;
        stiffness = s;
        UpdateLength();
        length0 = length;
        this.friction = friction;
        volume = 0;

    }
    public void UpdateLength()
    {
        length = (nodeA.pos - nodeB.pos).magnitude;
    }

    public void ComputeForces()
    {
        Vector3 u = nodeA.pos - nodeB.pos;
        u.Normalize();
        Vector3 force = - stiffness * (length - length0) * u;
        force += -friction * stiffness * Vector3.Dot(u, nodeA.vel - nodeB.vel) * u;
        nodeA.force += force;
        nodeB.force -= force;
    }
    
    public void ComputeForceWithDensity()
    {
        Vector3 u = nodeA.pos - nodeB.pos;
        Vector3 force = -volume / (length0 * length0) * stiffness * (length - length0) * u / length;
        u.Normalize();
        force += -friction * stiffness * Vector3.Dot(u, nodeA.vel - nodeB.vel) * u;

        nodeA.force += force;
        nodeB.force -= force;
    }
}
