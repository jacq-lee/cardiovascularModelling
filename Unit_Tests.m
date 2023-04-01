function tests = Unit_Tests
tests = functiontests(localfunctions);
end

function testPhaseCategorization(testCase)

circulation_model = Circulation(75, 2, 0.06);
test_states = [1; 2; 3; 4]; % model should be in filling phase since x2 > x1
test_time = 2;

actSolution = circulation_model.get_derivative(test_time, test_states);
expSolution = circulation_model.filling_phase_dynamic_matrix(test_time)*test_states;
verifyEqual(testCase,actSolution,expSolution)
end