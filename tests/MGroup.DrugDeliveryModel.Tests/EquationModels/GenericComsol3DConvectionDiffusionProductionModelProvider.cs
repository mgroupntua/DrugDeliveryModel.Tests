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

            
            return model;
        }


        /// <summary>
        /// gia to model workingtetmesh155 to 0.1, 0.1, 0.,1 kuvo 
        /// </summary>
        /// <param name="model"></param>
        /// <param name="maxZ">value =2</param>
        /// <param name="topValueprescribed"></param>
        /// <param name="minZ">value = 0</param>
        /// <param name="bottomValueprescribed"></param>
        /// <returns></returns>
        public void AddTopAndBottomBCs(Model model, double modelMaxZ, double topValueprescribed, double modelMinZ, double bottomValueprescribed)
        {
            
            var topNodes = new List<INode>();
            var bottomNodes = new List<INode>();
            var innerBulkNodes = new List<INode>();
            foreach (var node in model.NodesDictionary.Values)
            {
                if (Math.Abs(modelMaxZ - node.Z) < 1E-9) topNodes.Add(node);
                else if (Math.Abs(modelMinZ - node.Z) < 1E-9) bottomNodes.Add(node);
                else innerBulkNodes.Add(node);
            }

            var dirichletBCs = new List<NodalUnknownVariable>();
            foreach (var node in topNodes)
            {
                dirichletBCs.Add(new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, topValueprescribed));            
            }
            foreach (var node in bottomNodes)
            {
                dirichletBCs.Add(new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, bottomValueprescribed));
            }

            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                dirichletBCs,
                new INodalConvectionDiffusionNeumannBoundaryCondition[] { }
            ));

        }

        public void AddInitialConditionsForTheRestOfBulkNodes(Model model, double modelMaxZ, double modelMinZ, double prescribedInitialConditionValueForBulk)
        {

            var topNodes = new List<INode>();
            var bottomNodes = new List<INode>();
            var innerBulkNodes = new List<INode>();
            foreach (var node in model.NodesDictionary.Values)
            {
                if (Math.Abs(modelMaxZ - node.Z) < 1E-9) topNodes.Add(node);
                else if (Math.Abs(modelMinZ - node.Z) < 1E-9) bottomNodes.Add(node);
                else innerBulkNodes.Add(node);
            }

            var intitalConditions = new List<INodalConvectionDiffusionInitialCondition>();
            foreach (var node in innerBulkNodes)
            {
                if ((!topNodes.Contains(node)) && (!bottomNodes.Contains(node)))
                {
                    intitalConditions.Add(new NodalInitialUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, prescribedInitialConditionValueForBulk));
                }

            }


            model.InitialConditions.Add(new ConvectionDiffusionInitialConditionSet(intitalConditions,
                new DomainInitialUnknownVariable[]
                { }));

            
        }



    }
}
