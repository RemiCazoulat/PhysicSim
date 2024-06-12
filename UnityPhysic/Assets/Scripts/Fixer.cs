using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour
{
    // Possibilities of the Fixer

    public bool IsInside(Vector3 p)
    { 
        return  GetComponent<Collider>().bounds.Contains(p);
    }
    
}
