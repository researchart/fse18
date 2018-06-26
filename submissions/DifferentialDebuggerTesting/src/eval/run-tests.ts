import { ChromeBrowser } from "../debuggers/chrome";
import { delay } from "../utils/promise";
import { assert, assertNever, compareNumbers, equalsArray, lines } from "../utils/utils";
import { FirefoxBrowser } from "../debuggers/firefox";
import { randomArrayElement, randomTrue, shuffle } from "../utils/random";
import {
    Breakpoint,
    Location,
    NotRunningState,
    ResumptionAction,
    Variable
} from "../debuggers/interface";
import { testcases, testcasesPath } from "./testcases";
import * as fs from "fs";
import { Difference, GlobalResults, NoDifference, Parameters, RunResults } from "./results";
import seedrandom = require("seedrandom");

const maxSeed = 1;
const paramsValues = [
    new Parameters(0.1, 0.2, 20),
];

async function main() {
    await Promise.all([ChromeBrowser.start(), FirefoxBrowser.start()]);

    for (const params of paramsValues) {
        const globalResults = new GlobalResults(params, "appendHeader");
        for (let testcase of testcases) {
            const scriptSource = fs.readFileSync(`${testcasesPath}/${testcase}.js`, "utf-8");
            for (let seed = 0; seed < maxSeed; seed++) {
                const rng = seedrandom(seed);

                const chrome = await ChromeBrowser.debugScript(scriptSource);
                const firefox = await FirefoxBrowser.debugScript(scriptSource);
                assert(chrome.scripts.length === 1 && firefox.scripts.length === 1,
                    "not exactly one script");
                assert(chrome.scripts[0].source === firefox.scripts[0].source,
                    "different script sources");

                let run = new RunResults(testcase, seed, scriptSource, globalResults);
                console.log(run.trace.path);

                // step 1 a) and b): set at least 1 breakpoint, remove some random breakpoints
                const scriptLines = lines(scriptSource);
                const maxBreakpoints = Math.max(1, Math.floor(params.breakpointsPerLine * scriptLines.length));
                const breakpointsToSet = shuffle<number>(rng, [...scriptLines.keys()]).slice(0, maxBreakpoints);
                for (const breakpointLine of breakpointsToSet) {
                    // step 1 a): set breakpoints
                    if (breakpointsLines(chrome.breakpoints()).has(breakpointLine)) {
                        continue;
                    }
                    run.trace.setBreakpoint(breakpointLine);
                    run.action("setBreakpoint");
                    const setBreakpoints = {
                        chrome: await chrome.setBreakpoint({
                            script: chrome.scripts[0], lineNumber: breakpointLine
                        }),
                        firefox: await firefox.setBreakpoint({
                            script: firefox.scripts[0], lineNumber: breakpointLine
                        })
                    };
                    run.trace.setBreakpointResult(setBreakpoints.chrome, chrome);
                    run.trace.setBreakpointResult(setBreakpoints.firefox, firefox);
                    if (!equalsBreakpoints(setBreakpoints.chrome, setBreakpoints.firefox)) {
                        run.finish(new Difference("breakpoint"), "setBreakpoint", breakpointLine);
                        break;
                    }
                    run.noDifference("breakpoint", "setBreakpoint", breakpointLine);

                    // step 1 b): remove just set breakpoints
                    if (randomTrue(rng, params.breakpointRemoveProb)) {
                        run.trace.appendLine("remove breakpoints from previous step");
                        run.action("removeBreakpoint");
                        const couldRemove = {
                            chrome: await chrome.removeBreakpoints(setBreakpoints.chrome),
                            firefox: await firefox.removeBreakpoints(setBreakpoints.firefox)
                        };
                        run.trace.removeBreakpointsResult(couldRemove.chrome, chrome);
                        run.trace.removeBreakpointsResult(couldRemove.firefox, firefox);
                        if (!equalsBreakpoints(chrome.breakpoints(), firefox.breakpoints())
                            || !equalsArray(setBreakpoints.chrome, couldRemove.chrome)
                            || !equalsArray(setBreakpoints.firefox, couldRemove.firefox)) {
                            run.finish(new Difference("breakpoint"), "removeBreakpoint", breakpointLine);
                            break;
                        }
                        run.noDifference("breakpoint", "removeBreakpoint", breakpointLine);
                    }
                }

                // step 2: resumption actions
                let pause: { chrome?: NotRunningState; firefox?: NotRunningState } = {};
                let lastAction: ResumptionAction | "startExecution" = "startExecution";
                let lastLine = -1;

                function onNotRunning(browser: string) {
                    return async function (pauseState: NotRunningState) {
                        pause[browser] = pauseState;

                        // wait until both are paused and have called this callback
                        if (pause.chrome !== undefined && pause.firefox !== undefined) {
                            run.trace.actionResult(pause.chrome, chrome);
                            run.trace.actionResult(pause.firefox, firefox);

                            if (bothFinished(pause.chrome, pause.firefox)) {
                                run.finish(new NoDifference("executionFinished"), lastAction, lastLine);
                            } else if (onlyOneFinished(pause.chrome, pause.firefox)) {
                                run.finish(new Difference("termination"), lastAction, lastLine);
                            } else
                            // both running
                            if (pause.chrome !== "finished" && pause.firefox !== "finished") {
                                if (!equalsLocation(pause.chrome.location, pause.firefox.location)) {
                                    run.finish(new Difference("pauseLocation"), lastAction, lastLine);
                                    return;
                                }
                                lastLine = pause.chrome.location.lineNumber;
                                run.noDifference("pauseLocation", lastAction, lastLine);

                                const callStack = {
                                    chrome: await chrome.callStackFunctionNames(pause.chrome),
                                    firefox: await firefox.callStackFunctionNames(pause.firefox)
                                };
                                run.trace.appendLine("  call stack");
                                run.trace.callStackResult(callStack.chrome, chrome);
                                run.trace.callStackResult(callStack.firefox, firefox);
                                if (!equalsCallStacks(callStack.chrome, callStack.firefox)) {
                                    run.finish(new Difference("callStack"), lastAction, lastLine);
                                    return;
                                }
                                run.noDifference("callStack", lastAction, lastLine);

                                const variables = {
                                    chrome: await chrome.nonGlobalVariables(pause.chrome),
                                    firefox: await firefox.nonGlobalVariables(pause.firefox),
                                };
                                run.trace.appendLine("  variables");
                                run.trace.variablesResult(variables.chrome, chrome);
                                run.trace.variablesResult(variables.firefox, firefox);
                                if (!equalsVariables(variables.chrome, variables.firefox)) {
                                    run.finish(new Difference("variables"), lastAction, lastLine);
                                    return;
                                }
                                run.noDifference("variables", lastAction, lastLine);

                                // generate next action, take it in both
                                if (run.actionCount() < params.maxActions) {
                                    const nextAction: ResumptionAction = randomArrayElement(rng, ["resume", "stepIn", "stepOut", "stepOver"] as ResumptionAction[]);
                                    run.action(nextAction);
                                    lastAction = nextAction;
                                    pause = {}; // reset
                                    await Promise.all([
                                        chrome.action(nextAction),
                                        firefox.action(nextAction)
                                    ]);
                                } else {
                                    run.finish(new NoDifference("maxActions"), lastAction, lastLine);
                                }
                            } else assertNever("either both, one, or none finished, what is this?");
                        }
                    }
                }

                if (!run.isFinished()) {
                    chrome.onNotRunning(onNotRunning("chrome"));
                    firefox.onNotRunning(onNotRunning("firefox"));

                    run.action(lastAction);
                    await Promise.all([chrome.startExecution(), firefox.startExecution()]);
                    // busy wait until all actions are run or finished
                    while (!run.isFinished()) {
                        await delay(100);
                    }
                }
                await Promise.all([chrome.close(), firefox.close()]);
            }
        }
    }

    await Promise.all([ChromeBrowser.close(), FirefoxBrowser.close()]);
}

function bothFinished(a: NotRunningState, b: NotRunningState): boolean {
    return a === "finished" && b === "finished";
}

function onlyOneFinished(a: NotRunningState, b: NotRunningState): boolean {
    return (a === "finished" && b !== "finished") || (a !== "finished" && b === "finished");
}

function equalsLocation(a: Location, b: Location): boolean {
    return a.lineNumber === b.lineNumber;
}

function breakpointsLines(bps: Breakpoint[]): Set<number> {
    return new Set(bps.map(bp => bp.actualLocation.lineNumber));
}

function equalsBreakpoints(a: Breakpoint[], b: Breakpoint[]) {
    return equalsArray(
        [...breakpointsLines(a)].sort(compareNumbers),
        [...breakpointsLines(b)].sort(compareNumbers));
}

function equalsCallStacks(a: string[], b: string[]): boolean {
    return equalsArray(a, b);
}

function equalsVariables(a: Variable[], b: Variable[]) {
    return equalsArray(a, b, (b1, b2) => b1.toString() === b2.toString());
}

//noinspection JSIgnoredPromiseFromCall
main();
