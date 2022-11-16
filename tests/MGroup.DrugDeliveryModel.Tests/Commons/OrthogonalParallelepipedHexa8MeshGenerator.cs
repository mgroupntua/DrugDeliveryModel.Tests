using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;

namespace MGroup.DrugDeliveryModel.Tests.Commons
{
    public class OrthogonalParallelepipedHexa8MeshGenerator
    {
        /// <summary>
        /// The number of nodes at x direction
        /// </summary>
        private int NNX { get; }

        /// <summary>
        /// The number of elements at x direction
        /// </summary>
        private int NElX => NNX - 1;

        /// <summary>
        /// The number of nodes at y direction
        /// </summary>
        private int NNY { get; }

        /// <summary>
        /// The number of elements at y direction
        /// </summary>
        private int NElY => NNY - 1;

        /// <summary>
        /// The number of nodes at z direction
        /// </summary>
        private int NNZ { get; }

        /// <summary>
        /// The number of elements at z direction
        /// </summary>
        private int NElZ => NNZ - 1;

        /// <summary>
        /// Total nodes of the mesh
        /// </summary>
        private int TotalNodes => NNX * NNX * NNZ;

        /// <summary>
        /// Total elements of th mesh
        /// </summary>
        private int TotalElements => NElX * NElY * NElZ;

        /// <summary>
        /// Domain Legth at x Direction
        /// </summary>
        private double Lx { get; }

        /// <summary>
        /// Domain Length at y Direction
        /// </summary>
        private double Ly { get; }

        /// <summary>
        /// Domain Length at Z direction
        /// </summary>
        private double Lz { get; }

        /// <summary>
        /// Maximum X coordinate of subdomain 0
        /// </summary>
        public double MaxX { get; }

        /// <summary>
        /// Maximum Y coordinate of subdomain 0
        /// </summary>
        public double MaxY { get; }

        /// <summary>
        /// Maximum Z coordinate of subdomain 0
        /// </summary>
        public double MaxZ { get; }

        public Dictionary<int, Node> Nodes=> Create3dDomainNodes();


        /// <summary>
        /// HEXA Element Connectivity with subdomains. Key : Element id. Value : Tuple of (Element Type, array of nodes, subdomain id) 
        /// </summary>
        public Dictionary<int, Tuple<CellType, Node[], int>> ElementConnectivity;

        /// <summary>
        /// HEXA Element Connectivity. Key : Tuple of (Element ID, Element Type) Value : List of Node IDs
        /// </summary>
        //public Dictionary<int, Tuple<CellType, Node[]>> ElementConnectivityWithoutSubdomains => Create3dDomainElementsWithoutSubdomains();


        public OrthogonalParallelepipedHexa8MeshGenerator(int nnx, int nny, int nnz, double lx, double ly, double lz)
        {
            NNX = nnx;
            NNY = nny;
            NNZ = nnz;
            Lx = lx;
            Ly = ly;
            Lz = lz;
            MeshPrinter();
        }


        public OrthogonalParallelepipedHexa8MeshGenerator(int nnx, int nny, int nnz, double lx, double ly, double lz, double maxX, double maxY, double maxZ)
        {
            NNX = nnx;
            NNY = nny;
            NNZ = nnz;
            Lx = lx;
            Ly = ly;
            Lz = lz;
            MaxX = maxX;
            MaxY = maxY;
            MaxZ = maxZ;
            ElementConnectivity = Create3dDomainElementsWithSubdomains(MaxX, MaxY, MaxZ);
            MeshPrinter();
        }

        private Dictionary<int, Node> Create3dDomainNodes()
        {
            var nodes = new Dictionary<int, Node>();
            var nodeID = 0;
            for (int k = 0; k < NNZ; k++)
            {
                for (int j = 0; j < NNY; j++)
                {
                    for (int i = 0; i < NNX; i++)
                    {
                        var x = i * Lx / (NNX - 1);
                        var y = j * Ly / (NNY - 1);
                        var z = k * Lz / (NNZ - 1);
                        nodes.Add(nodeID, new Node(id: nodeID, x: x, y: y, z: z));
                        nodeID++;
                    }
                }
            }
            return nodes;
        }


        public Dictionary<int, Tuple<CellType, Node[], int>> Create3dDomainElementsWithSubdomains(double maxX, double maxY, double maxZ)
        {
            var elementConnectivity = new Dictionary<int, Tuple<CellType, Node[], int>>();
            var elementID = 0;
            for (int k = 0; k < NElZ; k++)
            {
                for (int j = 0; j < NElY; j++)
                {
                    for (int i = 0; i < NElX; i++)
                    {
                        var nodeID0 = i + j * NNX + k * NNX * NNY;
                        var nodeID1 = nodeID0 + 1;
                        var nodeID2 = nodeID1 + NNX;
                        var nodeID3 = nodeID0 + NNX;
                        var nodeID4 = nodeID0 + NNX * NNY;
                        var nodeID5 = nodeID1 + NNX * NNY;
                        var nodeID6 = nodeID2 + NNX * NNY;
                        var nodeID7 = nodeID3 + NNX * NNY;
                        var nodes = new Node[] { Nodes[nodeID6], Nodes[nodeID7], Nodes[nodeID4], Nodes[nodeID5], Nodes[nodeID2], Nodes[nodeID3], Nodes[nodeID0], Nodes[nodeID1] };
                        var subdomainID = 0;

                        foreach (var node in nodes)
                        {
                            if (node.X > maxX || node.Y > maxY || node.Z > maxZ)
                            {
                                subdomainID = 1;
                                break;
                            }
                        }
                        elementConnectivity.Add(elementID, new Tuple<CellType, Node[], int>(CellType.Hexa8, nodes, subdomainID));
                        elementID++;
                    }
                }
            }
            return elementConnectivity;
        }

        private void MeshPrinter()
        {
            Console.WriteLine("Total Nodes: " + TotalNodes);
            Console.WriteLine("Total Elements: " + TotalElements);
            Console.WriteLine("Total Elements Subdomain 0: " + ElementConnectivity.Where(x => x.Value.Item3 == 0).Count());
            Console.WriteLine("Total Elements Subdomain 1: " + ElementConnectivity.Where(x => x.Value.Item3 == 1).Count());
            Console.WriteLine("ID \t X \t Y \t Z");
            foreach (var node in Nodes)
            {
                Console.WriteLine(node.Key + "\t" + node.Value.X + "\t" + node.Value.Y + "\t" + node.Value.Z);
            }
            foreach (var element in ElementConnectivity)
            {
                Console.WriteLine("Element: " + element.Key + " Domain: " + element.Value.Item3);
                foreach (var node in element.Value.Item2)
                {
                    Console.WriteLine("\tNode {0}: ({1}, {2}, {3})", node.ID, node.X, node.Y, node.Z);
                }

            }
        }


    }
}
