# Crash2Mesh
Read time-variant HDF5 crash test simulation results, extract surfaces as triangle meshes and decimate to acceptable size for VR visualization

# Assumptions made about input (WIP)
- Connectivities are time-invariant
- Finite set of element types (no new user defined elements will be appearing):
    - BAR (=line2)
    - BEAM (=line2)
    - HEXA8
    - PENTA6
    - PLINK (=line2)
    - SHELL (= quad4)
    - NODE
- All elements specified in order as described in https://myesi.esi-group.com/ERF-HDF5/doc/ERF_CSM_RESULT_Specs_1.2.pdf
- time-variant Node positions always saved under Translational_Displacement
- ...
