import pytest
from carton.folding_solver import FoldingSolver


@pytest.mark.parametrize('parameters',[
    {'timestep':0}, {'tolerance':float('nan')}, {'impratio':-1},
    {'noslip_iterations':-1}, {'noslip_iterations':4},
    {'noslip_iterations':1.5}, {'noslip_iterations':True},
])
def test_invalid_solver_experiments_are_rejected(parameters):
    with pytest.raises(ValueError):FoldingSolver(**parameters)
