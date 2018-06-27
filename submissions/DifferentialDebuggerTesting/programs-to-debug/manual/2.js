function foo() { return 3; }

for (var i = 0; i < 3; i++) {
	console.log(i);
}

for (let i = 0; i < 3; i++) {
	console.log(i);
}

for (let i = 0; i < foo(); i++) {
	console.log(i);
}