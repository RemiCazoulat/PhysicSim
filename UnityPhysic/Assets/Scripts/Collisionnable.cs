using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collisionnable : MonoBehaviour
{
    public float rigidness;

    private Bounds myBounds;

    private void Awake()
    {
        myBounds = GetComponent<Collider>().bounds;
    }
    
   
    private bool IsInside(Vector3 p)
    {
        
        //var result = GetComponent<Collider>().bounds.Contains(p);
        var result =  myBounds.Contains(p);
        return result;
    }

    public Vector3 PenaltyForce(Vector3 pos)
    {
        if (!IsInside(pos))
        {
            return Vector3.zero;
        }
        //var closestPoint = GetComponent<Collider>().bounds.ClosestPoint(pos);
        
        var normal = pos - myBounds.center;
        var amplitude = myBounds.extents.magnitude / normal.magnitude;
        normal.Normalize();
        var force = rigidness * amplitude * normal; 

        return force;
    }

    public Matrix3X3 ImplicitIntegration(Node n, float timeStep)
    {
        if (!IsInside(n.pos)) return Matrix3X3.Id();
        

        //var closestPoint = GetComponent<Collider>().bounds.ClosestPoint(n.pos);
        var closestPoint = myBounds.ClosestPoint(n.pos);
        var forceDirection = closestPoint - n.pos;
        forceDirection.Normalize();
        
        var force0DerivativeConst = timeStep * timeStep / n.mass * rigidness;
        var force0Derivative = new Matrix3X3(forceDirection, forceDirection);
        force0Derivative.MultFloat(force0DerivativeConst);
        var id = new Matrix3X3(new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1));
        var sub = id - force0Derivative;
        sub.Inverse();
        return sub;
    }
}
