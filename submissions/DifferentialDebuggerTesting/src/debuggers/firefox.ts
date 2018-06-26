import fs = require("fs-extra");
import EventEmitter = require("events");
import { spawn, spawnSync } from "child_process";
import { FirefoxRemoteProtocol, Source, Tab, Thread } from "./firefox-protocol";
import {
    Breakpoint,
    Browser,
    DebugSession,
    Location,
    NotRunningState,
    Script,
    Variable
} from "./interface";
import { delay } from "../utils/promise";
import { assert, assertNever, remove } from "../utils/utils";
import "../utils/flat-map";

let firefoxRunsSinceLastRestart = 0;
const maxFirefoxRunsUntilRestart = 50;

export const FirefoxBrowser: Browser = {
    async start(): Promise<void> {
        spawn("yarn", ["firefox"]);
        await delay(10000);
    },

    async close(): Promise<void> {
        spawnSync("wmctrl", ["-c", "Mozilla Firefox"]);
        await delay(5000);
        spawnSync("wmctrl", ["-c", "Mozilla Firefox"]);
        await delay(15000);
    },

    async debugScript(scriptSource: string): Promise<FirefoxDebugSession> {
        if (firefoxRunsSinceLastRestart++ >= maxFirefoxRunsUntilRestart) {
            await FirefoxBrowser.close();
            await FirefoxBrowser.start();
            firefoxRunsSinceLastRestart = 0;
        }

        const tmpFile = Math.random().toString(36).substr(2, 5);
        fs.ensureDirSync("/tmp/automatic-debugger-testing");
        fs.writeFileSync(`/tmp/automatic-debugger-testing/${tmpFile}.js`, scriptSource);
        fs.writeFileSync(`/tmp/automatic-debugger-testing/${tmpFile}.html`, `<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>minimal JavaScript browser HTML page</title>
    <script src="${tmpFile}.js"></script>
  </head>
  <body>
    minimal JavaScript browser HTML page
  </body>
</html>`);

        const ff = await FirefoxRemoteProtocol.connect();
        let tabs = await ff.listTabs();
        let tab = tabs[0];

        await ff.navigateTo(tab, `file:///tmp/automatic-debugger-testing/${tmpFile}.html`);
        let thread = await ff.attachAndPause(tab);

        let firefoxScripts = [] as Script[];
        const sources = await ff.sources(thread);
        for (let source of sources) {
            // do not include scripts that lack a URL
            if (source.url === null) continue;
            firefoxScripts.push(Object.assign(source, {
                id: source.actor,
                source: await ff.getSource(source)
            }));
        }

        return new FirefoxDebugSession(ff, tab, thread, firefoxScripts, this);
    },

    toString() {
        return "Firefox";
    }
};

export class FirefoxDebugSession extends DebugSession {

    private events = new EventEmitter();

    constructor(private readonly ff: FirefoxRemoteProtocol,
                private readonly tab: Tab,
                private readonly thread: Thread,
                readonly scripts: Script[],
                readonly browser: Browser) {
        super(browser, scripts);
        // console.log(JSON.stringify(tab, null, 2));
        assert(scripts.length > 0, "firefox: tab has no scripts");
        this.ff.on(thread.actor, { type: "paused" }, pauseState => this.events.emit("pause", pauseState));
    }

    async close(): Promise<void> {
        await this.ff.detachAndRun(this.tab);
        // await this.ff.navigateTo(this.tab, "about:blank");
        this.ff.close();
    }

    async setBreakpoint(location: Location): Promise<Breakpoint[]> {
        let firefoxBreakpoint: any = await this.ff.request(location.script.id, "setBreakpoint", {}, {
            location: { line: location.lineNumber + 1 }, noSliding: false
        });

        // ensure that actual location (with column) is always present and 0-indexed
        if (!firefoxBreakpoint.hasOwnProperty("actualLocation")) {
            firefoxBreakpoint.actualLocation = location;
        } else {
            firefoxBreakpoint.actualLocation = toCanonicalLocation(firefoxBreakpoint.actualLocation);
        }
        firefoxBreakpoint.id = firefoxBreakpoint.actor;

        this.breakpoints_.push(firefoxBreakpoint);
        return [firefoxBreakpoint];
    }

    async removeBreakpoint(bp: Breakpoint): Promise<boolean> {
        assert(remove(this.breakpoints_, bp),
            "firefox: could not find breakpoint-to-remove in internal list of breakpoints");
        return Promise.race([
            this.ff.once((bp as any).actor, { error: "noSuchActor" }).then(() => false),
            this.ff.request((bp as any).actor, "delete", {}).then(() => true)
        ]);
    }

    async startExecution(): Promise<void> {
        await this.ff.on(this.tab.actor, {
            type: "tabNavigated",
            state: "stop"
        }, () => this.events.emit("tabNavigated"));
        await this.ff.request(this.tab.actor, "reload", {});
        await delay(500);
    }

    async resume(): Promise<void> {
        await this.ff.request(this.thread.actor, "resume", { type: "resumed" }, { resumeLimit: null });
    }

    async stepIn(): Promise<void> {
        await this.ff.request(this.thread.actor, "resume", { type: "resumed" }, { resumeLimit: { type: "step" } });
    }

    async stepOut(): Promise<void> {
        await this.ff.request(this.thread.actor, "resume", { type: "resumed" }, { resumeLimit: { type: "finish" } });
    }

    async stepOver(): Promise<void> {
        await this.ff.request(this.thread.actor, "resume", { type: "resumed" }, { resumeLimit: { type: "next" } });
    }

    onNotRunning(callback: (pauseState: NotRunningState) => void): void {
        this.events.on("pause", (firefoxPauseState: any) => {
            let pauseState = firefoxPauseState;
            pauseState.breakpointIds = (firefoxPauseState.why.actors === undefined) ? [] : firefoxPauseState.why.actors;
            pauseState.location = toCanonicalLocation(pauseState.frame.where);
            callback(pauseState);
        });
        this.events.on("tabNavigated", () => {
            callback("finished");
        });
    }

    async callStackFunctionNames(pauseState: NotRunningState): Promise<string[]> {
        const { frames } = await this.ff.request(this.thread.actor, "frames", { frames: null as any });
        return frames.filter(f => f.callee !== undefined)
        // use displayName for e.g. prototype functions (name is undefined for the anonymous
        // function that is assigned, but displayName is)
            .map(f => f.callee.name || f.callee.displayName)
            .filter(s => s !== undefined && s.length > 0)
            // Chrome shows prototype fns without .prototype, so remove it here
            .map(s => s.replace(".prototype", ""));
    }

    async nonGlobalVariables(pauseState: NotRunningState): Promise<Variable[]> {
        const environments = nestedEnvironmentsToArray((pauseState as any).frame.environment);
        const variables = environments.flatMap(env => firefoxBindingsToVariables(env.bindings));
        const sortedUniqueBindings = Variable.takeFirst(variables);
        return Promise.resolve(sortedUniqueBindings);
    }
}

function toCanonicalLocation(firefoxLocation: { line: number, column?: number, source: Source }): Location {
    return {
        script: { id: firefoxLocation.source.actor, url: firefoxLocation.source.url },
        // line should be 0-based
        lineNumber: firefoxLocation.line - 1,
        columnNumber: (firefoxLocation.column === undefined) ? undefined : firefoxLocation.column - 1
    };
}

// Firefox nests environments but we want a linear array
// see http://searchfox.org/mozilla-central/source/devtools/docs/backend/protocol.md#lexical-environments
function nestedEnvironmentsToArray(currentEnv): any[] {
    let envArray = [] as any[];
    // last environment has no parent == global env -> leave out
    while (currentEnv.parent !== undefined) {
        envArray.push(currentEnv);
        currentEnv = currentEnv.parent;
    }
    return envArray;
}

// Firefox splits between arguments (given as an array, because in JS args with the same name
// are allowed :D) and variables (as properties) -.-
function firefoxBindingsToVariables(firefoxBindingsObject): Variable[] {
    assert(firefoxBindingsObject.arguments !== undefined
        && firefoxBindingsObject.variables !== undefined, "not a firefox bindings object");

    // merge .arguments and .variables in one array
    let bindings = firefoxBindingsObject.arguments as any[];
    for (let variable in firefoxBindingsObject.variables) {
        bindings.push({ [variable]: firefoxBindingsObject.variables[variable] });
    }

    // remove special arguments object (that Firefox shows, but Chrome doesn't)
    bindings = bindings.filter(bindingObject => !(
        bindingObject.hasOwnProperty("arguments")
        && bindingObject.arguments.value.class !== undefined
        && bindingObject.arguments.value.class === "Arguments"
    ));

    // firefox descriptions to own Variable type
    return bindings.flatMap(gripToBinding);
}

// see http://searchfox.org/mozilla-central/source/devtools/docs/backend/protocol.md#grips
function gripToBinding(firefoxGrip): Variable[] {
    // console.log(JSON.stringify(firefoxGrip, null, 2)); // DEBUG

    assert(Object.getOwnPropertyNames(firefoxGrip).length === 1, "not a firefox grip");
    const name = Object.getOwnPropertyNames(firefoxGrip)[0];
    const firefoxValue = firefoxGrip[name].value;
    assert(firefoxValue !== undefined, "not a firefox grip");

    if (firefoxValue.optimizedOut) {
        return [];
    }

    switch (firefoxValue.type) {
        // primitives
        case undefined:
            if (typeof firefoxValue === "string") {
                return [new Variable(name, typeof firefoxValue, firefoxValue.slice(0, 30))];
            }
            return [new Variable(name, typeof firefoxValue, firefoxValue)];
        case "longString":
            return [new Variable(name, "string", firefoxValue.initial.slice(0, 30))];
        // special cases because not expressible in JSON
        case "null":
            return [new Variable(name, "null")];
        case "undefined":
            return [new Variable(name, "undefined")];
        case "Infinity":
        case "-Infinity":
        case "NaN":
        case "-0":
            return [new Variable(name, "number", firefoxValue.type)];
        // functions, array, general objects
        case "object":
            switch (firefoxValue.class) {
                case "Function":
                    return [new Variable(name, "function")];
                case "Array":
                    return [new Variable(name, "array")];
                default:
                    return [new Variable(name, "object")];
            }
    }
    return assertNever(`unkown grip type/value ${JSON.stringify(firefoxGrip)}`);
}
