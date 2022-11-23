using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.FEM.Helpers;
using MGroup.DrugDeliveryModel.Tests.Commons;
using MGroup.Constitutive.Structural.InitialConditions;
using MGroup.Constitutive.ConvectionDiffusion.InitialConditions;

namespace MGroup.DrugDeliveryModel.Tests.EquationModels
{
    public class GenericComsol3DConvectionDiffusionProductionModelProvider
    {
        public double[] ConvectionCoeff;  //=> new[]  {1d, 1d, 1d};
        public double DiffusionCoeff;
        public double DependentProductionCoeff;
        public double IndependentProductionCoeff;
        private double CapacityCoeff;

        public ComsolMeshReader reader { get; private set; }

        public GenericComsol3DConvectionDiffusionProductionModelProvider(double[] convectionCoeff,
            double diffusionCoeff, double dependentProductionCoeff, double independentProductionCoeff, double capacityCoeff)
        {
            ConvectionCoeff = convectionCoeff;
            DiffusionCoeff = diffusionCoeff;
            DependentProductionCoeff = dependentProductionCoeff;
            IndependentProductionCoeff = independentProductionCoeff;
            CapacityCoeff = capacityCoeff;

        }

        public Model CreateModelFromComsolFile(string filename)
        {
            var model = new Model();
            model.SubdomainsDictionary[0] = new Subdomain(id: 0);

            reader = new ComsolMeshReader(filename);

            foreach (var node in reader.NodesDictionary.Values)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var material = new ConvectionDiffusionProperties(
                capacityCoeff: CapacityCoeff,
                diffusionCoeff: DiffusionCoeff,
                convectionCoeff: ConvectionCoeff,
                dependentSourceCoeff: DependentProductionCoeff,
                independentSourceCoeff: IndependentProductionCoeff);

            var elementFactory = new ConvectionDiffusionElement3DFactory(material);

            foreach (var elementConnectivity in reader.ElementConnectivity)
            {
                var element = elementFactory.CreateElement(elementConnectivity.Value.Item1, elementConnectivity.Value.Item2);
                model.ElementsDictionary.Add(elementConnectivity.Key, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            var topNodes = new List<INode>();
            var bottomNodes = new List<INode>();
            var innerBulkNodes = new List<INode>();
            foreach (var node in model.NodesDictionary.Values)
            {
                if (Math.Abs(2 - node.Z) < 1E-9) topNodes.Add(node);
                else if (Math.Abs(0 - node.Z) < 1E-9) bottomNodes.Add(node);
                else innerBulkNodes.Add(node);
            }

            int i = 0;
            var dirichletBCs = new NodalUnknownVariable[topNodes.Count + bottomNodes.Count];
            foreach (var node in topNodes)
            {
                dirichletBCs[i] = new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, 100d);
                i++;
            }
            foreach (var node in bottomNodes)
            {
                dirichletBCs[i] = new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, 50d);
                i++;
            }

            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                dirichletBCs,
                new INodalConvectionDiffusionNeumannBoundaryCondition[] { }
            ));

            var intitalConditions = new List<INodalConvectionDiffusionInitialCondition>();
            foreach (var node in model.NodesDictionary.Values)
            {
                if ((!topNodes.Contains(node))&& (!bottomNodes.Contains(node)))
                {
                    intitalConditions.Add(new NodalInitialUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, initialConditionAmount);
                }
                
            }

            
            //model.InitialConditions.Add(new StructuralInitialConditionSet(
            //    new[]
            //    {
            //        new NodalInitialDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, 0),
            //    },
            //    new[]
            //    {
            //        new DomainInitialVelocity(StructuralDof.TranslationZ, 1),
            //    }));

            return model;
        }

        public Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary[0] = new Subdomain(id: 0);

            int nodeIndex = -1;
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 2.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 1.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 0.0);
            model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 0.0);

            var elementNodes = new INode[][]
            {
                new[] { model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12], model.NodesDictionary[10], model.NodesDictionary[1], model.NodesDictionary[0], model.NodesDictionary[9] },
                new[] { model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13], model.NodesDictionary[11], model.NodesDictionary[2], model.NodesDictionary[1], model.NodesDictionary[10] },
                new[] { model.NodesDictionary[16], model.NodesDictionary[7], model.NodesDictionary[6], model.NodesDictionary[15], model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12] },
                new[] { model.NodesDictionary[17], model.NodesDictionary[8], model.NodesDictionary[7], model.NodesDictionary[16], model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13] },
                new[] { model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21], model.NodesDictionary[19], model.NodesDictionary[10], model.NodesDictionary[9], model.NodesDictionary[18] },
                new[] { model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22], model.NodesDictionary[20], model.NodesDictionary[11], model.NodesDictionary[10], model.NodesDictionary[19] },
                new[] { model.NodesDictionary[25], model.NodesDictionary[16], model.NodesDictionary[15], model.NodesDictionary[24], model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21] },
                new[] { model.NodesDictionary[26], model.NodesDictionary[17], model.NodesDictionary[16], model.NodesDictionary[25], model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22] },
            };

            var nodeReordering = new GMeshElementLocalNodeOrdering();
            var rearrangeNodes = elementNodes.Select(x => nodeReordering.ReorderNodes(x, CellType.Hexa8)).ToArray();

            var material = new ConvectionDiffusionProperties(
                capacityCoeff: 0d,
                diffusionCoeff: DiffusionCoeff,
                convectionCoeff: ConvectionCoeff,
                dependentSourceCoeff: DependentProductionCoeff,
                independentSourceCoeff: IndependentProductionCoeff);

            var elementFactory = new ConvectionDiffusionElement3DFactory(material);
            for (int i = 0; i < elementNodes.Length; i++)
            {
                model.ElementsDictionary[i] = elementFactory.CreateElement(CellType.Hexa8, rearrangeNodes[i]);
                model.ElementsDictionary[i].ID = i;
                model.SubdomainsDictionary[0].Elements.Add(model.ElementsDictionary[i]);
            }
;
            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new[]
                {
                    new NodalUnknownVariable(model.NodesDictionary[0], ConvectionDiffusionDof.UnknownVariable,  100d),
                    new NodalUnknownVariable(model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable,  100d),
                    new NodalUnknownVariable(model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable,  100d),
                    new NodalUnknownVariable(model.NodesDictionary[9], ConvectionDiffusionDof.UnknownVariable,  100d),
                    new NodalUnknownVariable(model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(model.NodesDictionary[11], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(model.NodesDictionary[18], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(model.NodesDictionary[19], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(model.NodesDictionary[20], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable,  50d),
                    new NodalUnknownVariable(model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable,  50d),
                    new NodalUnknownVariable(model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable,  50d),
                    new NodalUnknownVariable(model.NodesDictionary[15], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(model.NodesDictionary[16], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(model.NodesDictionary[17], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(model.NodesDictionary[24], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(model.NodesDictionary[25], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(model.NodesDictionary[26], ConvectionDiffusionDof.UnknownVariable, 50d),
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[] { }
            ));

            return model;
        }


    }
}
