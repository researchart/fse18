const scriptConst = 3.14;

fnName("arg1", "arg2", "arg3");
function fnName(param, param2, param) {

    var shadowedVarBound = 1337;
    {
    	var shadowedVarBound = 12;

        let LetInNestedBlockBound = 42;
    }

    {
        let secondBlock = 1;
    }

    var varBound = 1337;
    let letBound = "hello world";
    const constBound = true;

    var notInitialized;

	var someObject = { bla: "test" };
    var someArray = [1, { bla: "test" }, "string" ];
	var someFunction = function() {};
	var someNull = null;
	var someUndefined = undefined;
	var someNotJsonSerializable1 = -0;
	var someNotJsonSerializable2 = NaN;
}