# MQSim Edited.


## Usage in Linux
Run following commands:
	
```
$ make
$ ./MQSim -i <SSD Configuration File> -w <Workload Definition File>
```

## Usage in Windows

1. Open the MQSim.sln solution file in MS Visual Studio 2017 or later.
2. Set the Solution Configuration to Release (it is set to Debug by default).
3. Compile the solution.
4. Run the generated executable file (e.g., MQSim.exe) either in command line mode or by clicking the MS Visual Studio run button. Please specify the paths to the files containing the 1) SSD configurations, and 2) workload definitions.

Example command line execution:

```
$ MQSim.exe -i <SSD Configuration File> -w <Workload Definition File> 
```

## Edited point.
1. memory leak problem is solved
2. precondition of MQ Sim operates, when Ideal mapping setting of sdd configration is ideal. 
3. FLIN Algorithm[3] is partially implemented till 2 stage. you can use FLIN policy of TSU.
4. CIF Algorithm for improving overhead of FLIN Algorithm is added. you can use CIF policy of TSU.  


## MQSim Execution Configurations 
You can get information from https://github.com/CMU-SAFARI/MQSim 

## MQSim Workload Definition
You can get information from https://github.com/CMU-SAFARI/MQSim 

## Analyze MQSim's XML Output
You can get information from https://github.com/CMU-SAFARI/MQSim 

## References
[1] A. Tavakkol et al., "MQSim: A Framework for Enabling Realistic Studies of Modern Multi-Queue SSD Devices," FAST, pp. 49 - 66, 2018.

[2] M. Jung and M. T. Kandemir, "Sprinkler: Maximizing Resource Utilization in Many-chip Solid State Disks," HPCA, pp. 524-535, 2014.

[3] A. Tavakkol et al , " FLIN: Enabling Fairness and Enhancing Performance in Modern NVMeSolid State Drives ", in IEEE 45thAnnual ISCA,pp.397-410 2018
