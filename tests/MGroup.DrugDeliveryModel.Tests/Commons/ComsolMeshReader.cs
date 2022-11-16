using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;

namespace MGroup.DrugDeliveryModel.Tests.Commons
{
    internal enum ReadingStatus
    {
        Unknown,
        FoundNodes,
        ReadingNodes,
        FoundTet4,
        ReadingTet4,
        FoundTet4Indices,
        ReadingTet4Indices,
        FoundWedge6,
        ReadingWedge6,
        FoundWedge6Indices,
        ReadingWedge6Indices,
        FoundHexa8,
        ReadingHexa8,
        FoundHexa8Indices,
        ReadingHexa8Indices
    }

    public class ComsolMeshReader
    {
        public Dictionary<int, Node> NodesDictionary { get; set; }
        public Dictionary<int, Tuple<CellType, Node[], int>> ElementConnectivity { get; set; }
        private Dictionary<int, Tuple<CellType, Node[], int>> ElementConnectivityTet4 { get; set; }
        private Dictionary<int, Tuple<CellType, Node[], int>> ElementConnectivityWedge6 { get; set; }
        private Dictionary<int, Tuple<CellType, Node[], int>> ElementConnectivityHexa8 { get; set; }

        public ComsolMeshReader(string filepath)
        {
            NodesDictionary = new Dictionary<int, Node>();
            ElementConnectivity = new Dictionary<int, Tuple<CellType, Node[], int>>();
            ElementConnectivityTet4 = new Dictionary<int, Tuple<CellType, Node[], int>>();
            ElementConnectivityWedge6 = new Dictionary<int, Tuple<CellType, Node[], int>>();
            ElementConnectivityHexa8 = new Dictionary<int, Tuple<CellType, Node[], int>>();

            try
            {
                using (var sr = new StreamReader(filepath))
                {
                    ReadingStatus status = ReadingStatus.Unknown;
                    ReadingStatus prevStatus = ReadingStatus.Unknown;
                    Console.WriteLine("Reading file {0}", filepath);
                    var line = sr.ReadLine();
                    var id = 0;
                    while (line != null)
                    {
                        //Status update 
                        if (line.Equals("# Mesh vertex coordinates"))
                        {//Nodes
                            prevStatus = status;
                            status = ReadingStatus.FoundNodes;
                            Console.WriteLine("Status: Found nodes");
                        }
                        else if (status == ReadingStatus.FoundNodes)
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingNodes;
                            Console.WriteLine("Status: Reading nodes");
                        }
                        else if (line.Equals("3 tet # type name"))
                        {//Tet4
                            prevStatus = status;
                            status = ReadingStatus.FoundTet4;
                            Console.WriteLine("Status: Found Tet4");
                        }
                        else if (status == ReadingStatus.FoundTet4 && !line.Contains("#") && !line.Equals(""))
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingTet4;
                            Console.WriteLine("Status: Reading Tet4");
                        }
                        else if (line.Equals("5 prism # type name"))
                        {//Wedge6
                            prevStatus = status;
                            status = ReadingStatus.FoundWedge6;
                            Console.WriteLine("Status: Found Wedge6");
                        }
                        else if (status == ReadingStatus.FoundWedge6 && !line.Contains("#") && !line.Equals(""))
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingWedge6;
                            Console.WriteLine("Status: Reading Wedge6");
                        }
                        else if (line.Equals("3 hex # type name"))
                        {//Hexa8
                            prevStatus = status;
                            status = ReadingStatus.FoundHexa8;
                            Console.WriteLine("Status: Found Hexa8");
                        }
                        else if (status == ReadingStatus.FoundHexa8 && !line.Contains("#") && !line.Equals(""))
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingHexa8;
                            Console.WriteLine("Status: Reading Hexa8");
                        }
                        else if (line.Contains("# Geometric entity indices"))
                        {//Geometric entity indices
                            if (prevStatus == ReadingStatus.ReadingTet4)
                            {
                                prevStatus = status;
                                status = ReadingStatus.FoundTet4Indices;
                                Console.WriteLine("Status: Found Tet4 Indices");
                            }
                            else if (prevStatus == ReadingStatus.ReadingWedge6)
                            {
                                prevStatus = status;
                                status = ReadingStatus.FoundWedge6Indices;
                                Console.WriteLine("Status: Found Wedge6 Indices");
                            }
                            else if (prevStatus == ReadingStatus.ReadingHexa8)
                            {
                                prevStatus = status;
                                status = ReadingStatus.FoundHexa8Indices;
                                Console.WriteLine("Status: Found Hexa8 Indices");
                            }
                        }
                        else if (status == ReadingStatus.FoundTet4Indices && !line.Contains("#") && !line.Equals(""))
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingTet4Indices;
                            id = ElementConnectivityTet4.First().Key;
                            Console.WriteLine("Status: Reading Tet4 Indices");
                        }
                        else if (status == ReadingStatus.FoundWedge6Indices && !line.Contains("#") && !line.Equals(""))
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingWedge6Indices;
                            id = ElementConnectivityWedge6.First().Key;
                            Console.WriteLine("Status: Reading Wedge6 Indices");
                        }
                        else if (status == ReadingStatus.FoundHexa8Indices && !line.Contains("#") && !line.Equals(""))
                        {
                            prevStatus = status;
                            status = ReadingStatus.ReadingHexa8Indices;
                            id = ElementConnectivityHexa8.First().Key;
                            Console.WriteLine("Status: Reading Hexa8 Indices");
                        }
                        else if (line.Equals("") &&
                                status != ReadingStatus.Unknown &&
                                status != ReadingStatus.FoundHexa8 &&
                                status != ReadingStatus.FoundTet4 &&
                                status != ReadingStatus.FoundWedge6)
                        {//Unknown
                            if (status == ReadingStatus.ReadingNodes) id = 0;
                            prevStatus = status;
                            status = ReadingStatus.Unknown;
                            Console.WriteLine("Status: Unknown");
                        }

                        //Action
                        if (status == ReadingStatus.ReadingNodes)
                        {//Nodes
                            //Split line
                            var coordsString = line.Split(" ");
                            //Convert to double
                            var coords = new double[coordsString.GetLength(0) - 1];
                            for (int i = 0; i < coords.Length; i++)
                                coords[i] = double.Parse(coordsString[i]);

                            NodesDictionary.Add(key: id, new Node(id: id, x: coords[0], y: coords[1], z: coords[2]));

                            //Print
                            string identation = id < 10 ? " " : "";
                            Console.WriteLine("Node {0}{1}: ({2}, {3}, {4})", id, identation, coords[0].ToString("F2"), coords[1].ToString("F2"), coords[2].ToString("F2"));
                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingTet4)
                        {
                            //Split line
                            var nodesString = line.Split(" ");
                            //Convert to int
                            var nodeIDs = new int[nodesString.GetLength(0) - 1];
                            for (int i = 0; i < nodeIDs.Length; i++)
                                nodeIDs[i] = int.Parse(nodesString[i]);
                            //Identify nodes
                            var nodes = new Node[4];
                            for (int i = 0; i < nodes.Length; i++)
                                nodes[i] = NodesDictionary[nodeIDs[i]];
                            ElementConnectivityTet4.Add(key: id, value: new Tuple<CellType, Node[], int>(CellType.Tet4, nodes, 0));

                            //Print
                            Console.WriteLine("Element {0}", id);
                            for (int i = 0; i < nodeIDs.Length; i++)
                            {
                                string identation = nodeIDs[i] < 10 ? " " : "";
                                Console.WriteLine("\tNode {0}{1}: ({2}, {3}, {4})", nodeIDs[i], identation, NodesDictionary[nodeIDs[i]].X.ToString("F5"), NodesDictionary[nodeIDs[i]].Y.ToString("F5"), NodesDictionary[nodeIDs[i]].Z.ToString("F5"));
                            }
                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingTet4Indices)
                        {
                            //Convert to int
                            var targetDomain = int.Parse(line) - 1;
                            var newTupple = new Tuple<CellType, Node[], int>(ElementConnectivityTet4[id].Item1, ElementConnectivityTet4[id].Item2, targetDomain);
                            ElementConnectivityTet4[id] = newTupple;

                            Console.WriteLine("\tElement {0} - Domain: {1}", newTupple.Item1, newTupple.Item3);

                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingWedge6)
                        {
                            //Split line
                            var nodesString = line.Split(" ");
                            //Convert to int
                            var nodeIDs = new int[nodesString.GetLength(0) - 1];
                            for (int i = 0; i < nodeIDs.Length; i++)
                                nodeIDs[i] = int.Parse(nodesString[i]);
                            //Identify nodes and reorder to match MSolve convention
                            var nodes = new Node[6];
                            var reorderedNodes = new int[] { 1, 2, 0, 4, 5, 3 };
                            //var reorderedNodes = new int[] { 0, 1, 2, 3, 4, 5 };

                            for (int i = 0; i < nodes.Length; i++)
                                nodes[reorderedNodes[i]] = NodesDictionary[nodeIDs[i]];
                            ElementConnectivityWedge6.Add(key: id, value: new Tuple<CellType, Node[], int>(CellType.Wedge6, nodes, 0));

                            //Print
                            Console.WriteLine("Element {0}", id);
                            for (int i = 0; i < nodeIDs.Length; i++)
                            {
                                string identation = nodeIDs[i] < 10 ? " " : "";
                                Console.WriteLine("\tNode {0}{1}: ({2}, {3}, {4})", nodeIDs[i], identation, NodesDictionary[nodeIDs[i]].X.ToString("F14"), NodesDictionary[nodeIDs[i]].Y.ToString("F14"), NodesDictionary[nodeIDs[i]].Z.ToString("F14"));
                            }
                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingWedge6Indices)
                        {
                            //Convert to int
                            var targetDomain = int.Parse(line) - 1;
                            var newTupple = new Tuple<CellType, Node[], int>(ElementConnectivityWedge6[id].Item1, ElementConnectivityWedge6[id].Item2, targetDomain);
                            ElementConnectivityWedge6[id] = newTupple;

                            Console.WriteLine("\tElement {0} - Domain: {1}", newTupple.Item1, newTupple.Item3);

                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingHexa8)
                        {
                            //Split line
                            var nodesString = line.Split(" ");
                            //Convert to int
                            var nodeIDs = new int[nodesString.GetLength(0) - 1];
                            for (int i = 0; i < nodeIDs.Length; i++)
                                nodeIDs[i] = int.Parse(nodesString[i]);
                            //Identify nodes and reorder to match MSolve convention
                            var nodes = new Node[8];
                            var reorderedNodes = new int[] { 6, 7, 5, 4, 2, 3, 1, 0 };
                            for (int i = 0; i < nodes.Length; i++)
                                nodes[reorderedNodes[i]] = NodesDictionary[nodeIDs[i]];
                            ElementConnectivityHexa8.Add(key: id, value: new Tuple<CellType, Node[], int>(CellType.Hexa8, nodes, 0));

                            //Print
                            Console.WriteLine("Element {0}", id);
                            //for (int i = 0; i < nodeIDs.Length; i++)
                            //{
                            //    string identation = nodeIDs[i] < 10 ? " " : "";
                            //    Console.WriteLine("\tNode {0}{1}: ({2}, {3}, {4})", nodeIDs[i], identation, NodesDictionary[nodeIDs[i]].X.ToString("F2"), NodesDictionary[nodeIDs[i]].Y.ToString("F2"), NodesDictionary[nodeIDs[i]].Z.ToString("F2"));
                            //}
                            foreach (var node in nodes)
                                Console.WriteLine("\tNode {0}{1}: ({2}, {3}, {4})", node.ID, " ", node.X.ToString("F2"), node.Y.ToString("F2"), node.Z.ToString("F2"));
                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingHexa8Indices)
                        {
                            //Convert to int
                            var targetDomain = int.Parse(line) - 1;
                            var newTupple = new Tuple<CellType, Node[], int>(ElementConnectivityHexa8[id].Item1, ElementConnectivityHexa8[id].Item2, targetDomain);
                            ElementConnectivityHexa8[id] = newTupple;

                            Console.WriteLine("\tElement {0} - Domain: {1}", newTupple.Item1, newTupple.Item3);

                            //Increment id
                            id++;
                        }

                        //Read next line
                        line = sr.ReadLine();
                    }
                }
            }
            catch (IOException e)
            {
                Console.WriteLine("The file could not be read:");
                Console.WriteLine(e.Message);
            }
            FlattenDictionaries();
            Console.WriteLine("Finished reading file");
        }

        private void FlattenDictionaries()
        {
            foreach(var entry in ElementConnectivityTet4)
                ElementConnectivity.Add(entry.Key, entry.Value);

            foreach (var entry in ElementConnectivityWedge6)
                ElementConnectivity.Add(entry.Key, entry.Value);
            
            foreach (var entry in ElementConnectivityHexa8)
                ElementConnectivity.Add(entry.Key, entry.Value);

        }
    }
}
