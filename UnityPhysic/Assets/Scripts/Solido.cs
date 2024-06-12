using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using TreeEditor;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Serialization;

public class Solido : MonoBehaviour
{
    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
    }
    public TextAsset solidNodesPos;
    public TextAsset solidTetraIndexes;
    public float timeStep;
    public int substep;
    public bool paused;
    public Integration integrationMethod;
    public List<Fixer> fixers;
    public List<Collisionnable> colliders;
    public bool floor;
    public float floorRigidness;
    public Vector3 gravity;
    public float density;
    public float springsStiffness;
    public float airFriction;
    public float verticesSize;

    public Vector3 windDirection;
    public float windStrength;
    public float windVariationSpeed;
    public float windVariation;
    public bool implicitIntegration;

    [Button(nameof(ResetAll))]
    public bool buttonField0;
    
    [Button(nameof(CreateSolid))]
    public bool buttonField1;
    [Button(nameof(LinkSolidWithMesh))]
    public bool buttonField2;


    private Mesh mesh;
    private Vector3[] solidVertices;
    private List<Tetrahedron> tetrahedrons;
    private List<Triangle> solidTriangles;
    private int[] whichTetra;
    private Vector4[] weights;
    private Node[] solidNodes;
    private List<Spring> springs;
    private Dictionary<String, int> existingSprings;
    private int[] whichFixer;
    private Vector3[] fixersPos;
    private Vector3[] newFixersPos;
    private float realTimeStep;
    private Parser parser = new();
    private float currentTime;
    private double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;

    #region buttonsFunctions
    public void ResetAll()
    { 
        mesh = null; 
        solidVertices = null;
        tetrahedrons = null;
        whichTetra = null;
        weights = null;
        solidNodes = null;
        springs = null;
        existingSprings = null;
        whichFixer = null;
        fixersPos = null;
        newFixersPos = null;
    }
    public void CreateSolid()
    {
        CreateSolidVertices();
        CreateTetrahedrons();
        CreateSolidTriangles();
        CreateFixers();
        TetrahedronsToNodesAndSprings();
    }
    
    public void LinkSolidWithMesh()
    {
        mesh = this.GetComponentInChildren<MeshFilter>().sharedMesh;
        AssignMeshVerticesToTetrahedron();
    }
    #endregion
    #region MonoBehaviour
    private void Start()
    {
        parser = new Parser();
       
        CreateSolid();
        LinkSolidWithMesh();
        AssignMeshVerticesToTetrahedron();

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
        for (int node = 0; node < solidNodes.Length; node++)
        {
            if (solidNodes[node].isFixed && fixersMoved[whichFixer[node]])
            {
                solidNodes[node].pos += fixersMovement[whichFixer[node]];
            }
            solidVertices[node] = solidNodes[node].pos;
        }
        NodesAndSpringsToTetrahedrons();
        MoveMesh();
        // Loop to update the fixers positions.
        for (int i = 0; i < fixers.Count; i++)
        {
            fixersPos[i] = newFixersPos[i];
        }
        
    }
    #region FixedUpdate
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
        foreach(var node in solidNodes)
        {
            if (node.isFixed) continue;
            node.force = Vector3.zero;
            node.ComputeGravity(gravity);
            PenaltyForce(node);

        }
        StepWind();

        // Setting springs forces
        foreach (var s in springs)
        {
            s.ComputeForceWithDensity();
        }
        // Calculate the new position of nodes
        foreach(var node in solidNodes)
        {
            node.Friction(airFriction);
            if (node.isFixed) continue;
            
            ImplicitIntegration(node); 
            node.pos += realTimeStep * node.vel;
            /*
            node.vel += realTimeStep / node.mass * node.force;
            node.pos += realTimeStep * node.vel;
            */
        }
        // Updating Lengths of springs
        foreach (var s in springs)
        {
            s.UpdateLength();
        }
        
    }
    private void StepWind()
    {
        if (currentTime > 2 * PI) currentTime = 0f;
        else currentTime += realTimeStep;
        float stepWindStrength = windStrength + (float)Math.Cos(currentTime * windVariationSpeed) * windVariation;
        foreach (var tri in solidTriangles)
        {
            int i1 = tri.i1;
            int i2 = tri.i2;
            int i3 = tri.i3;
            Vector3 ab = solidVertices[i2] - solidVertices[i1];
            Vector3 ac = solidVertices[i3] - solidVertices[i1];
            Vector3 normTri = Vector3.Cross(ab, ac);
            normTri.Normalize();
                
            Vector3 velTri = 1f / 3f * solidNodes[i1].vel +
                             solidNodes[i2].vel + solidNodes[i3].vel;
            float surfaceTri = ab.magnitude * ac.magnitude / 2;
            Vector3 windForce = airFriction * surfaceTri *
                                Vector3.Dot(normTri, stepWindStrength * windDirection - velTri) * normTri;
            windForce /= 3f;
            solidNodes[i1].force += windForce;
            solidNodes[i2].force += windForce;
            solidNodes[i3].force += windForce;
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
        
        if (!implicitIntegration)
        {
            var vel1 = Matrix3X3.MatVecMultiply(derivatives, speedAndForce);
            node.vel = vel1;
            return;
        }
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
    #endregion
    #region Gizmos
    
    private void OnDrawGizmosSelected()
    {
       GizmosDrawing();
       
        
    }

    private void OnDrawGizmos()
    {
        GizmosDrawing();
    }
    

    private void GizmosDrawing()
    {
        Gizmos.color = Color.blue;
        if (solidNodes != null)
        {
            foreach (var node in solidNodes)
            {
                Gizmos.DrawSphere(node.pos, verticesSize);
            }
        }
        if (mesh != null)
        {
            for (int vertex = 0; vertex < mesh.vertices.Length; vertex ++)
            {
                Vector3 globalPos = transform.TransformPoint(mesh.vertices[vertex]);
                if (whichTetra != null && whichTetra[vertex] == 0)
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawSphere(globalPos, verticesSize / 2);
                    continue;
                }
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(globalPos, verticesSize / 2);
            }
        }
        Gizmos.color = Color.red;
        if (springs != null)
        {
            foreach(var spring in springs)
            {
                Gizmos.DrawLine(spring.nodeA.pos, spring.nodeB.pos);
            }
        }

        Gizmos.color = Color.magenta;
        if (tetrahedrons != null && solidVertices != null)
        {
            Tetrahedron t = tetrahedrons[0];
            Gizmos.DrawSphere( t.p1, verticesSize * 1.5f);
            Gizmos.DrawSphere( t.p2, verticesSize * 1.5f);
            Gizmos.DrawSphere( t.p3, verticesSize * 1.5f);
            Gizmos.DrawSphere( t.p4, verticesSize * 1.5f);
        }
        /*
        Gizmos.color = Color.black;

        if (tetrahedrons != null)
        {
            foreach (var t in tetrahedrons)
            {
                Gizmos.DrawLine(t.midFacePos1, t.midFacePos1 + t.normFace1);
                Gizmos.DrawLine(t.midFacePos2, t.midFacePos2 + t.normFace2);
                Gizmos.DrawLine(t.midFacePos3, t.midFacePos3 + t.normFace3);
                Gizmos.DrawLine(t.midFacePos4, t.midFacePos4 + t.normFace4);

            }
        }
        */
        
    }
    #endregion
    #endregion
    private void CreateFixers()
    {
        whichFixer = new int[solidVertices.Length];
        fixersPos = new Vector3[fixers.Count];
        newFixersPos = new Vector3[fixers.Count];
        for(int i = 0; i < fixers.Count; i ++ )
        {
            fixersPos[i] = fixers[i].transform.position;
            newFixersPos[i] = fixersPos[i];
        }
    }
    private void CreateSolidVertices()
    {
        if (solidNodesPos == null || solidTetraIndexes == null)
        {
            Debug.LogError("Please insert a file for modelNode and modelEle");
            return;
        }
        
        float[] rawVertices = parser.ParseFloat(solidNodesPos, true);
        solidVertices = new Vector3[(int)rawVertices[0]];
        int verticesIndex = 0;
        Transform t = transform;
        for (int i = 4; i < rawVertices.Length; i += 4)
        {
            Vector3 vertex = new Vector3(rawVertices[i + 1], rawVertices[i + 2], rawVertices[i + 3]);
            //var globalPos = transform.TransformPoint(vertex);
            solidVertices[verticesIndex] = t.rotation * vertex + t.position ;
            verticesIndex++;
        }
        
        string debug = "Vertices debug : \n";
        for (int i = 0; i < solidVertices.Length; i++)
        {
            debug += "vertex " + i + " : " + solidVertices[i] + "\n";
        }
        Debug.Log(debug);
    }
    private void CreateTetrahedrons()
    {
        tetrahedrons = new List<Tetrahedron>();
        
        if (solidNodesPos == null || solidTetraIndexes == null)
        {
            Debug.LogError("Please insert a file for modelNode and modelEle");
            return;
        }
        
        int[] rawTetrahedrons = parser.ParseInt(solidTetraIndexes, true);

        for (var i = 3; i < rawTetrahedrons.Length; i += 5)
        {
            
            int i1 = rawTetrahedrons[i + 1] - 1;
            int i2 = rawTetrahedrons[i + 2] - 1;
            int i3 = rawTetrahedrons[i + 3] - 1;
            int i4 = rawTetrahedrons[i + 4] - 1;

            Vector3 p1 = solidVertices[i1];
            Vector3 p2 = solidVertices[i2];
            Vector3 p3 = solidVertices[i3];
            Vector3 p4 = solidVertices[i4];

            tetrahedrons.Add(new Tetrahedron(p1, p2, p3, p4, i1, i2, i3, i4, density));
            
        }
        string debug = "tetrahedrons debug : \n";
        for (int i = 0; i < tetrahedrons.Count; i++)
        {
            Tetrahedron t = tetrahedrons[i];
            debug += "tetrahedron " + i + " : " + t.i1 + ", " + t.i2 + ", " +
                     t.i3 + ", " + t.i4 + "\n";
        }
        Debug.Log(debug);
    }

    private void CreateSolidTriangles()
    {
        List<Triangle> allTris = new List<Triangle>();
        List<int> triCount = new List<int>();
        foreach (var t in tetrahedrons)
        {
            int i1 = t.i1;
            int i2 = t.i2;
            int i3 = t.i3;
            int i4 = t.i4;

            Triangle tri1 = new Triangle(i1, i2, i3);
            Triangle tri2 = new Triangle(i1, i2, i4);
            Triangle tri3 = new Triangle(i1, i3, i4);
            Triangle tri4 = new Triangle(i2, i3, i4);
            Triangle[] tris = { tri1, tri2, tri3, tri4};
            foreach (var tri in tris)
            {
                bool exists = false;
                for (int i = 0; i < allTris.Count; i++)
                {
                    if (allTris[i].ID == tri.ID)
                    {
                        triCount[i] += 1;
                        exists = true;
                        break;
                    }
                }
                if (exists) continue;
                allTris.Add(tri);
                triCount.Add(1);
            }
        }
        solidTriangles = new List<Triangle>();
        for (int i = 0; i < allTris.Count; i++)
        {
            if (triCount[i] == 1)
            {
                solidTriangles.Add(allTris[i]);
            }
        }
        
    }
    private void TetrahedronsToNodesAndSprings()
    {
        solidNodes = new Node[solidVertices.Length];
        // ----{ SETTING NODES }----
        for (int i = 0; i < solidVertices.Length; i++)
        {
            Vector3 pos = solidVertices[i];
            solidNodes[i] = new Node(pos);
            whichFixer[i] = IsInside(pos);
            solidNodes[i].isFixed = whichFixer[i] != -1;
        }
        // ----{ SETTING SPRINGS }----
        existingSprings = new Dictionary<string, int>();
        springs = new List<Spring>();
        int springIndex = 0;
        for (int i = 0; i < tetrahedrons.Count; i ++)
        {
            Tetrahedron t = tetrahedrons[i]; 
            int index0 = t.i1;
            int index1 = t.i2;
            int index2 = t.i3;
            int index3 = t.i4;
            solidNodes[index0].mass += t.mass / 3;
            solidNodes[index1].mass += t.mass / 3;
            solidNodes[index2].mass += t.mass / 3;
            solidNodes[index3].mass += t.mass / 3;

            string spring01 = index0 < index1 ? index0 + index1.ToString() : index1 + index0.ToString(); 
            string spring02 = index0 < index2 ? index0 + index2.ToString() : index2 + index0.ToString(); 
            string spring03 = index0 < index3 ? index0 + index3.ToString() : index3 + index0.ToString(); 
            string spring12 = index1 < index2 ? index1 + index2.ToString() : index2 + index1.ToString(); 
            string spring13 = index1 < index3 ? index1 + index3.ToString() : index3 + index1.ToString(); 
            string spring23 = index2 < index3 ? index2 + index3.ToString() : index3 + index2.ToString();
            string[] springsID = {spring01, spring02, spring03, spring12, spring13, spring23};
            int[]    indexes0  = {index0  , index0  , index0  , index1  , index1  , index2};
            int[]    indexes1  = {index1  , index2  , index3  , index2  , index3  , index3};
            for(int j = 0; j < 6; j ++)
            {
                if (!SpringExists(springsID[j]))
                {
                    springs.Add(new Spring(solidNodes[indexes0[j]], solidNodes[indexes1[j]], springsStiffness, airFriction));
                    existingSprings.Add(springsID[j], springIndex);
                    springIndex++;
                }
                int index = existingSprings[springsID[j]];
                springs[index].volume += t.volume / 6;
            }
        }
    }
    private void NodesAndSpringsToTetrahedrons()
    {
        foreach (var t in tetrahedrons)
        {
            t.p1 = solidVertices[t.i1];
            t.p2 = solidVertices[t.i2];
            t.p3 = solidVertices[t.i3];
            t.p4 = solidVertices[t.i4];
            t.RecalculateMass(density);
        }
    }
    private void AssignMeshVerticesToTetrahedron()
    {
        string debug = "[DEBUG] Vertices in tetrahedrons : \n";
        
        whichTetra = new int[mesh.vertices.Length];
        weights = new Vector4[mesh.vertices.Length];
        for (int vertex = 0; vertex < mesh.vertices.Length; vertex++)
        {
            debug += "vertex " + vertex + " is in tetrahedron "; 
            Vector3 pos = mesh.vertices[vertex];
            Vector3 globalPos = transform.TransformPoint(pos);

            for (int tetra = 0; tetra < tetrahedrons.Count; tetra++)
            {
                Tetrahedron t = tetrahedrons[tetra];
                Vector3 p1 = t.p1;
                Vector3 p2 = t.p2;
                Vector3 p3 = t.p3;
                Vector3 p4 = t.p4;
                Vector3 normFace1 = t.normFace1;
                Vector3 normFace2 = t.normFace2;
                Vector3 normFace3 = t.normFace3;
                Vector3 normFace4 = t.normFace4;
                
                float w1 = Vector3.Dot(normFace1, globalPos - p4) / Vector3.Dot(normFace1,   p1 - p4);
                if (w1 < 0) continue;
                //Debug.Log("1");
                float w2 = Vector3.Dot(normFace2, globalPos - p3) / Vector3.Dot(normFace2,   p2 - p3);
                if (w2 < 0) continue;
                //Debug.Log("2");
                float w3 = Vector3.Dot(normFace3, globalPos - p2) / Vector3.Dot(normFace3,   p3 - p2);
                if (w3 < 0) continue;
                //Debug.Log("3");
                float w4 = Vector3.Dot(normFace4, globalPos - p1) / Vector3.Dot(normFace4,   p4 - p1);
                if (w4 < 0) continue;
                //Debug.Log("4");
                weights[vertex] = new Vector4(w1, w2, w3, w4);
                whichTetra[vertex] = tetra;
                debug += tetra + ".";
                break;
            }

            debug += "\n";
        }
        Debug.Log(debug);
    }
    private void MoveMesh()
    {
        Vector3[] vertices = new Vector3[mesh.vertices.Length];
        for (int vertex = 0; vertex < mesh.vertices.Length; vertex++)
        {           
            int tetra = whichTetra[vertex];
            Tetrahedron t = tetrahedrons[tetra];
            Vector3 p1 = transform.InverseTransformPoint(t.p1);
            Vector3 p2 = transform.InverseTransformPoint(t.p2);
            Vector3 p3 = transform.InverseTransformPoint(t.p3);
            Vector3 p4 = transform.InverseTransformPoint(t.p4);

            Vector4 currentWeights = weights[vertex];
            float w1 = currentWeights[0];
            float w2 = currentWeights[1];
            float w3 = currentWeights[2];
            float w4 = currentWeights[3];

            vertices[vertex] = 
                w1 * p1 + 
                w2 * p2 + 
                w3 * p3 + 
                w4 * p4;
        }
        mesh.vertices = vertices;
    }
    #region utilities
   
    private bool SpringExists(string spring)
    {
        bool exists = false;
        foreach (var (s, i) in existingSprings)
        {
            if (spring.Equals(s))
            {
                exists = true;
                break;
            }
        }
        return exists;
    }
    private int IsInside(Vector3 p)
    {
        for (int i = 0; i < fixers.Count; i++)
        {
            if (fixers[i].IsInside(p)) return i;
        }
        return -1;
    }
    #endregion
}
