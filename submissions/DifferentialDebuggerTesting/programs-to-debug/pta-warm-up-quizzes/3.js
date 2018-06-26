var r = Math.random(); // value in [0,1)
var out = "yes";
if (r < 0.5)
out = "no";
if (r === 1)
out = "maybe"; // infeasible path
console.log(out);