function f(a,b) {
var x;
for (var i = 0; i < arguments.length; i++) {
x += arguments[i];
}
console.log(x);
}
f(1,2,3);