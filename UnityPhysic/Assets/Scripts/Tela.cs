using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;
using Random = System.Random;
public class Tela : MonoBehaviour
{


    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
    };

    #region InEditorVariables

    [Header ("---------- Simulation ---------- ")]
    [Space(5)]
    public float timeStep;
    public int substep;
    public bool paused;
    public Integration integrationMethod;
    [Space(10)]
    [Header ("---------- Outer forces & outer objects ----------")]
    [Space(5)]
    public List<Fixer> fixers;
    [Space(5)]
    public List<Collisionnable> colliders;
    [Space(5)]
    public bool floor;
    public float floorRigidness;
    public float floorFriction;
    [Space(5)]
    public Vector3 gravity;
    [Space(5)]
    public Vector3 windDirection;
    public float windStrength;
    public float windVariation;
    public float windVariationSpeed;
    public float airFriction;

    [Space(10)] [Header("---------- Tela parameters ----------")] [Space(5)]
    public Vector3 rotation;
    public bool planeBothSides;
    public uint height; // in meters 
    public uint width; // in meters
    public uint resolutionHeight; // vertices per columns
    public uint resolutionWidth; // vertices per ligns

    public float tractionSpringsStiffness;
    public float flexionSpringsStiffness;
    public float globalMass;
    [Space(10)]

    [Button(nameof(GeneratePlaneInEditMode))]
    public bool buttonField;
    /*
    [Button(nameof(GenerateNodesAndSpringsInEditMode))]
    public bool buttonField1;
    */

    #endregion

    #region OtherVariables

    private float realTimeStep;
    // Nodes variables
    private Node[] nodes;
    private int[] whichFixer;
    private Vector3[] fixersPos;
    private Vector3[] newFixersPos;
    // Springs variables
    private List<string> existingTractionSprings;
    private List<Spring> tractionSprings;
    private List<Spring> flexionSprings;
    // Mesh variables
    private Vector3[] vertices;
    private int[] triangles;
    private Mesh mesh;
    // Utilities variables 
    private float currentTime = 0f;

    #endregion
    
    #region Constants

    private double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;
    #endregion
    #region MonoBehaviour

  
    private void Start()
    {
        windDirection.Normalize();
        GeneratePlane(planeBothSides);
        GenerateFixers();
        GenerateNodesAndSprings();
    }

    

    private void Update()
    {
        bool[] fixersMoved = new bool[fixers.Count];
        Vector3[] fixersMovement = new Vector3[fixers.Count];
        // Loop to check if a fixer has moved.
        for(int i = 0; i < fixers.Count; i ++ )
        {
            newFixersPos[i] = fixers[i].transform.position;
            fixersMoved[i] = fixersPos[i] != newFixersPos[i];
            fixersMovement[i] = newFixersPos[i] - fixersPos[i];
        }
        // Loop to assign the new position of each nodes.
        for (int node = 0; node < vertices.Length; node++)
        {
            Vector3 localPos = transform.InverseTransformPoint(nodes[node].pos);
            if (nodes[node].isFixed && fixersMoved[whichFixer[node]])
            {
                nodes[node].pos += fixersMovement[whichFixer[node]];
            }
            vertices[node] = localPos;
        }
        // Loop to update the fixers positions.
        for (int i = 0; i < fixers.Count; i++)
        {
            fixersPos[i] = newFixersPos[i];
        }
        mesh.vertices = vertices;
    }
    
    public void FixedUpdate()
    {
        if (paused)
            return; // Not simulating
      
        
        // Select integration method.
        for (int i = 0; i < substep; i++)
        {
            switch (integrationMethod)
            {
                case Integration.Explicit: StepExplicit(); break;
                case Integration.Symplectic: StepSymplectic(); break;
                default:
                    throw new Exception("[ERROR] Should never happen!");
            }
        }
       

    }

    private void StepExplicit()
    {
        
    }
    private void StepSymplectic()
    {
        realTimeStep = timeStep / substep;
        // Setting forces
        foreach(var node in nodes)
        {
            if (node.isFixed) continue;
            node.force = Vector3.zero;
            node.ComputeGravity(gravity);
            PenaltyForce(node);
        }
        // Setting wind forces
        StepWind();
        // Setting traction springs forces
        foreach (var s in tractionSprings)
        {
            s.ComputeForces();
        }
        // Setting flexion springs forces
        foreach (var s in flexionSprings)
        {
            s.ComputeForces();
        }
        // Calculate the new position of nodes
        foreach(var node in nodes)
        {
            node.Friction(airFriction);
            if (node.isFixed) continue;
            ImplicitIntegration(node); 
            node.pos += realTimeStep * node.vel;
            
        }
        // Updating Lengths of springs
        foreach (var s in tractionSprings)
        {
            s.UpdateLength();
        }
        foreach (var s in flexionSprings)
        {
            s.UpdateLength();
        }
    }

    private void StepSymplecticGPU()
    {
        
    }

    private void StepWind()
    {
        if (currentTime > 2 * PI) currentTime = 0f;
        else currentTime += realTimeStep;

        float stepWindStrength = windStrength + (float)Math.Cos(currentTime * windVariationSpeed) * windVariation;
        int step = planeBothSides ? 12 : 6;
        for (int i = 0; i < triangles.Length; i += step)
        {
            for(int j = 0; j < 2; j += 3)
            {
                int index = i + j;
                Vector3 ab = vertices[triangles[index + 1]] - vertices[triangles[index]];
                Vector3 ac = vertices[triangles[index + 2]] - vertices[triangles[index]];
                Vector3 normTri = Vector3.Cross(ab, ac);
                normTri.Normalize();
                Vector3 velTri = 1f/3f * nodes[triangles[index]].vel + nodes[triangles[index + 1]].vel + nodes[triangles[index + 2]].vel;
                float surfaceTri = ab.magnitude * ac.magnitude / 2;
                Vector3 windForce = airFriction * surfaceTri * Vector3.Dot(normTri, stepWindStrength * windDirection - velTri) * normTri;
                windForce /= 3f;
                nodes[triangles[index    ]].force += windForce;
                nodes[triangles[index + 1]].force += windForce;
                nodes[triangles[index + 2]].force += windForce;
            }
        }
    }
    private void PenaltyForce(Node node)
    {
        //Floor first
        if (floor && node.pos.y < 0f)
        {
            float distance = 0f - node.pos.y;
            Vector3 direction = new Vector3(0f, 1f, 0f);
            Vector3 force0 = distance * floorRigidness * direction;
            node.force += force0;
        }
        foreach(var col in colliders)
        {
            node.force += col.PenaltyForce(node.pos);
        }
    }
    private void ImplicitIntegration(Node node)
    {
        var speedAndForce = node.vel + realTimeStep / node.mass * node.force;
        var derivatives = Matrix3X3.Id();
        //Floor first
        if (floor && node.pos.y < 0f)
        {
            var normal = new Vector3(0.0f, 1.0f, 0.0f);
            var force0DerivativeConst = realTimeStep * realTimeStep / node.mass * floorRigidness;
            var force0Derivative = new Matrix3X3(normal, normal);
            force0Derivative.MultFloat(force0DerivativeConst);
            var id = Matrix3X3.Id();
            var sub = id - force0Derivative;
            sub = sub.Inverse();
            derivatives = sub;
            //bool diff = finalVel != speedAndForce;
        }
        //Other colliders 
        foreach (var col in colliders)
        {
            derivatives = col.ImplicitIntegration(node, realTimeStep);
        }
        // Calculating new velocity
        var vel = Matrix3X3.MatVecMultiply(derivatives, speedAndForce);
        node.vel = vel;
    }
    
    /// <summary>
    /// Setting the fixers and their positions.
    /// </summary>
    private void GenerateFixers()
    {
        whichFixer = new int[vertices.Length];
        fixersPos = new Vector3[fixers.Count];
        newFixersPos = new Vector3[fixers.Count];
        for(int i = 0; i < fixers.Count; i ++ )
        {
            fixersPos[i] = fixers[i].transform.position;
            newFixersPos[i] = fixersPos[i];
        }
    }
    /// <summary>
    /// Create the nodes, the flexion springs, and the traction springs.
    /// </summary>
    private void GenerateNodesAndSprings()
    {
        
        nodes = new Node[vertices.Length];
        existingTractionSprings = new List<string>();
        tractionSprings = new List<Spring>();
        flexionSprings = new List<Spring>();
         // ----{ SETTING NODES }----
        float nodeMass = globalMass / nodes.Length;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 globalPos = transform.TransformPoint(vertices[i]);
            Vector3 pos = vertices[i];
            nodes[i] = new Node(pos, nodeMass);

            whichFixer[i] = IsInside(globalPos);
            nodes[i].isFixed = whichFixer[i] != -1;
        }

        // ----{ SETTING TRACTION SPRINGS }----
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int index0 = triangles[i + 0];
            int index1 = triangles[i + 1];
            int index2 = triangles[i + 2];
            // these strings are here to verify that the springs are not already created. 
            // This syntax is used to prevent errors because the string "01" is not the same that "10",
            // but the spring from node 0 to 1 or 1 to 0 is the same.
            string spring01 = index0 < index1 ? index0 + index1.ToString() : index1 + index0.ToString(); 
            string spring12 = index1 < index2 ? index1 + index2.ToString() : index2 + index1.ToString(); 
            string spring20 = index2 < index0 ? index2 + index0.ToString() : index0 + index2.ToString();
            // Now we verify the existence of each spring, and if it doesn't exist, we add a new spring in the traction spring list
            // and wi add the string associated in the string list used to verify the existence.
            if (!SpringExists(spring01))
            {
                tractionSprings.Add(new Spring(nodes[index0], nodes[index1], tractionSpringsStiffness, airFriction));
                existingTractionSprings.Add(spring01);
            }
            if (!SpringExists(spring12))
            {
                tractionSprings.Add(new Spring(nodes[index1], nodes[index2], tractionSpringsStiffness, airFriction));
                existingTractionSprings.Add(spring12);
            }
            if (!SpringExists(spring20))
            {
                tractionSprings.Add(new Spring(nodes[index2], nodes[index0], tractionSpringsStiffness, airFriction));
                existingTractionSprings.Add(spring20);
            }
        }
        
        // ----{ SETTING FLEXION SPRINGS }----
        for (int i = 0; i < triangles.Length / 6; i++)
        {
            int index0 = triangles[i * 6 + 0];
            int index1 = triangles[i * 6 + 4];
            flexionSprings.Add( new Spring(nodes[index0], nodes[index1], flexionSpringsStiffness, airFriction));
            //Debug.Log("[FLEXION] "+i+" : from " +i * 6 + " (pos : "+nodes[index0].pos+") to "+(i * 6 + 4)+" (pos : "+nodes[index1].pos+")");
        }
        //For simulation purposes, transform the points to global coordinates
        foreach (var node in nodes)
        {
            Vector3 globalPos = transform.TransformPoint(node.pos);
            node.pos = globalPos;
        }
    }

    /// <summary>
    /// Verify if a spring has already been created.
    /// </summary>
    /// <param name="spring">The name of the spring we want to check.</param>
    private bool SpringExists(string spring)
    {
        bool exists = false;
        foreach (var s in existingTractionSprings)
        {
            if (spring.Equals(s))
            {
                exists = true;
                break;
            }
        }

        return exists;
    }
    /// <summary>
    /// Create a Plane with a size of height x width and a number of vertices of (height * resolution) x (width * resolution).
    /// </summary>
    /// <param name="bothSides"> If true, create triangles on both sides of the plan. If false, only in one side.</param>
    private void GeneratePlane(bool bothSides)
    {
        var modifyRot = rotation * (float)PI / 180;
        var yaw = modifyRot.x % (2.0f * (float)PI);
        var pitch = modifyRot.y % (2.0f * (float)PI);
        var roll = modifyRot.z % (2.0f * (float)PI);
        var direction = new Vector3(1, 0, 0);
        var up = new Vector3(0, 1, 0);
        var right = new Vector3(0, 0, 1);
        var rotX = RotationX(yaw);
        var rotY = RotationY(pitch);
        var rotZ = RotationZ(roll);
        var rotationMatrix = MatMult( MatMult(rotZ, rotY), rotX);
        direction = MatVecMult(rotationMatrix, direction);
        up = MatVecMult(rotationMatrix, up);
        right = MatVecMult(rotationMatrix, right);
    
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;;

        Int32 xSize = (int)resolutionHeight;
        Int32 zSize = (int)resolutionWidth;
        float stepHeight =  resolutionHeight / (float)height;
        float stepWidth = resolutionWidth / (float)width;

        Int32 verticesSize = (xSize + 1) * (zSize + 1);
        vertices = new Vector3[verticesSize];
        Int32 i = 0;
        for(int z = 0; z < zSize + 1 ; z ++)
        {
            for (int x = 0; x < xSize + 1; x ++)
            {
                vertices[i] =  right * z / stepWidth + direction * x / stepHeight;
                //vertices[i] = new Vector3(  x / (float)resolution,  0, z / (float)resolution);
                i++;
            }
        }

        int triCount = bothSides ? 12 : 6;
        long trianglesSize = xSize * zSize * triCount;

        triangles = new Int32[trianglesSize];
        Int32 vert = 0;
        Int32 tris = 0;
        for (int z = 0; z < zSize; z++) {
            for (int x = 0; x < xSize; x++) {
                triangles[tris + 0] = vert;
                triangles[tris + 1] = vert + xSize + 1;
                triangles[tris + 2] = vert + 1;
                triangles[tris + 3] = vert + xSize + 1;
                triangles[tris + 4] = vert + xSize + 2;
                triangles[tris + 5] = vert + 1;
                if (bothSides)
                {
                    triangles[tris + 0 + 6] = vert + 1;
                    triangles[tris + 1 + 6] = vert + xSize + 1;
                    triangles[tris + 2 + 6] = vert;
                    triangles[tris + 3 + 6] = vert + 1;
                    triangles[tris + 4 + 6] = vert + xSize + 2;
                    triangles[tris + 5 + 6] = vert + xSize + 1;
                }
                vert++;
                tris += triCount;
            }
            vert++;
        }
        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
    }

    
    /// <summary>
    ///Check if a node has to be fixed or not with one of the fixers.
    /// </summary>
    /// <param name="p"> The position of the given node.</param>
    /// <returns></returns>
    private int IsInside(Vector3 p)
    {
        for (int i = 0; i < fixers.Count; i++)
        {
            if (fixers[i].IsInside(p)) return i;
        }
        return -1;
    }
    #endregion
    
       /// <summary>
    /// Calculs the rotation along the x axis of an angle.
    /// </summary>
    /// <param name="angle">the angle</param>
    /// <returns>the rotation matrix</returns>
    private static float[,] RotationX(float angle)
    {
        float cosAngle = (float)Math.Cos(angle);
        float sinAngle = (float)Math.Sin(angle);
        return new[,]
        {
            {1, 0, 0},
            {0, cosAngle, -sinAngle},
            {0, sinAngle, cosAngle}
        };
    }
    /// <summary>
    /// Calculs the rotation along the y axis of an angle.
    /// </summary>
    /// <param name="angle">the angle</param>
    /// <returns>the rotation matrix</returns>
    private static float[,] RotationY(double angle)
    {
        float cosAngle = (float)Math.Cos(angle);
        float sinAngle = (float)Math.Sin(angle);
        return new[,]
        {
            {cosAngle, 0, sinAngle},
            {0, 1, 0},
            {-sinAngle, 0, cosAngle}
        };
    }
    /// <summary>
    /// Calculs the rotation along the z axis of an angle.
    /// </summary>
    /// <param name="angle">the angle</param>
    /// <returns>the rotation matrix</returns>
    private static float[,] RotationZ(double angle)
    {
        float cosAngle = (float)Math.Cos(angle);
        float sinAngle = (float)Math.Sin(angle);
        return new[,]
        {
            {cosAngle, -sinAngle, 0},
            {sinAngle, cosAngle, 0},
            {0, 0, 1}
        };
    }
    /// <summary>
    /// Multiplicates 2 matrices.
    /// </summary>
    /// <param name="matrix1">the first matrix</param>
    /// <param name="matrix2">the seccond matrix</param>
    /// <returns>the multiplication between these two matrices.</returns>
    private static float[,] MatMult(float[,] matrix1, float[,] matrix2)
    {
        int rows = matrix1.GetLength(0);
        int cols = matrix2.GetLength(1);
        float[,] result = new float[rows, cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i, j] = 0;
                for (int k = 0; k < matrix1.GetLength(1); k++) {
                    result[i, j] += matrix1[i, k] * matrix2[k, j];
                }
            }
        }
        return result;
    }
    /// <summary>
    /// Multiplicates a vector with a matrix.
    /// </summary>
    /// <param name="matrix">the matrix</param>
    /// <param name="vector">the vector</param>
    /// <returns>the multiplication between the matrix and the vector</returns>
    private static Vector3 MatVecMult(float[,] matrix, Vector3 vector)
    {
        
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);
        Vector3 result = new();
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i] += matrix[i, j] * vector[j];
            }
        }
        return result;
    }
    public void GeneratePlaneInEditMode()
    {
        GeneratePlane(planeBothSides);
        
    }
}
