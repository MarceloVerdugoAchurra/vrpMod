const util = require('util');

var tap = require('tap');
var ortools = require('node_or_tools');


module.exports = {


    solver: function(locations, durationMatrix, costMatrix, callback) {
		
		
		var serviceTime = 0; //tiempo que se debe actualizar para la matriz "durationMatrix".
		
        var depot = 0;

        function manhattanDistance(lhs, rhs) {
            return Math.abs(lhs[0] - rhs[0]) + Math.abs(lhs[1] - rhs[1]);
        }

        var dayStarts = Hours(0);
        var dayEnds = Hours(3);

        var seed = 2147483650;

        function ParkMillerRNG(seed) {
            var modulus = 2147483647;
            var multiplier = 48271;
            var increment = 0;
            var state = seed;

            return function() {
                state = (multiplier * state + increment) % modulus;
                return state / modulus;
            };
        }

        var rand = ParkMillerRNG(seed);


        function Seconds(v) {
            return v;
        };

        function Minutes(v) {
            return Seconds(v * 60);
        }

        function Hours(v) {
            return Minutes(v * 60);
        }


        var timeWindows = new Array(locations.length);

        for (var at = 0; at < locations.length; ++at) {
            if (at === depot) {
                timeWindows[at] = [dayStarts, dayEnds];
                continue;
            }

            var earliest = dayStarts;
            var latest = dayEnds - Hours(1);

            /* var start = rand() * (latest - earliest) + earliest;
            var stop = rand() * (latest - start) + start; */
			
			var start = dayStarts
			var stop = dayEnds

            timeWindows[at] = [start, stop];
        }

		
        var demandMatrix = new Array(locations.length);

        for (var from = 0; from < locations.length; ++from) {
            demandMatrix[from] = new Array(locations.length);

            for (var to = 0; to < locations.length; ++to) {
                if (from === depot)
                    demandMatrix[from][to] = 0
                else
                    //demandMatrix[from][to] = parseInt(Math.random() * (20 - 5) + 5)
                    demandMatrix[from][to] = parseInt(Math.random() * (20 - 1) + 1)
            }
        }

        tap.test('Test VRP', function(assert) {

            var solverOpts = {
                numNodes: locations.length,
                costs: costMatrix,
                durations: durationMatrix,
                timeWindows: timeWindows,
                demands: demandMatrix
            };

            var VRP = new ortools.VRP(solverOpts);

            //var numVehicles = 6;
	        var numVehicles = 15;
            var timeHorizon = dayEnds - dayStarts;
            var vehicleCapacity = 30;

            // Dummy lock to let vehicle 0 go to location 2 and 3 first - to test route locks
            var routeLocks = new Array(numVehicles);

            for (var vehicle = 0; vehicle < numVehicles; ++vehicle) {
                if (vehicle === 0)
                    routeLocks[vehicle] = [2, 3];
                else
                    routeLocks[vehicle] = [];
            }

            var searchOpts = {
                computeTimeLimit: 1000,
                numVehicles: numVehicles,
                depotNode: depot,
                timeHorizon: timeHorizon,
                vehicleCapacity: vehicleCapacity,
                routeLocks: routeLocks,
                pickups: [],
                deliveries: []
            };

            VRP.Solve(searchOpts, function(err, solution) {
                assert.ifError(err, 'Solution can be found');

                assert.type(solution, Object, 'Solution is Object with properties');
                assert.type(solution.routes, Array, 'Routes in solution is Array with routes per vehicle');
                assert.type(solution.times, Array, 'Times in solution is Array with time windows per vehicle');

                assert.equal(solution.routes.length, numVehicles, 'Number of routes is number of vehicles');
                assert.equal(solution.times.length, numVehicles, 'Number of time windows in number of vehicles');

                function used(v) {
                    return v.length == 0 ? 0 : 1;
                }

                function addition(l, r) {
                    return l + r;
                }

                var numVehiclesUsed = solution.routes.map(used).reduce(addition, 0);

                assert.ok(numVehiclesUsed > 0, 'Solution uses vehicles');
                assert.ok(numVehiclesUsed <= numVehiclesUsed, 'Solution uses up to number of vehicles');

                function checkRoute(v) {
                    var depotInRoute = v.find(function(u) {
                        return u == depot;
                    });
                    assert.ok(!depotInRoute, 'Depot is not in route');
                }

                function checkTimeWindows(v) {
                    v.forEach(function(u) {
                        assert.ok(u[0] < u[1], 'Valid Time Window');
                    });
                }

                solution.routes.forEach(checkRoute);
                solution.times.forEach(checkTimeWindows);

                // We locked vehicle 0 to go to location 2 and 3 first
                assert.equal(solution.routes[0][0], 2);
                assert.equal(solution.routes[0][1], 3);

                assert.end();
				
				return callback(null, solution);
				
            });
        });


    }


}