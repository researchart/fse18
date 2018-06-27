import fs = require("fs-extra");
import ChromeImpl = require("chrome-remote-interface");
import { spawn, spawnSync } from "child_process";
import { delay, toPromise } from "../utils/promise";
import {
    Breakpoint,
    Browser,
    DebugSession,
    Location,
    NotRunningState,
    Script,
    Variable
} from "./interface";
import { assert, id, remove } from "../utils/utils";
import "../utils/flat-map";

export const ChromeBrowser: Browser = {
    async start(): Promise<void> {
        spawn("yarn", ["chrome"]);
        await delay(6000);
    },

    async close(): Promise<void> {
        spawnSync("pkill", ["-f", "chromium-browser"]);
    },

    async debugScript(scriptSource: string): Promise<ChromeDebugSession> {
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

        // open a new empty tab, connect to it and enable debugging + page event notifications
        const tabHandle = await ChromeImpl.New();
        const tab = await ChromeImpl({ tab: tabHandle });

        // NOTE wait to make sure about:blank has finished loading, otherwise we sometimes
        // got stuck somewhere around the pause/navigate/paused block below -.-
        await delay(100);

        let scripts = [] as any[];
        tab.Debugger.scriptParsed(script => scripts.push(script));

        // NOTE on Debugger.enable scriptParsed events are sent for all loaded scripts, but because
        // we opened about:blank (which doesn't have any scripts) it will never fire before the
        // Page.navigate call below
        await tab.Debugger.enable();
        await tab.Page.enable();

        // pause right after navigate so that Chrome slides breakpoints (if not in paused it will not)
        await tab.Debugger.pause();
        // navigate to trigger scriptParsed events
        await tab.Page.navigate({ url: `file:///tmp/automatic-debugger-testing/${tmpFile}.html` });
        await toPromise(tab.Debugger.paused);

        let chromeScripts = [] as Script[];
        for (let script of scripts) {
            chromeScripts.push(Object.assign(script, {
                id: script.scriptId,
                source: (await tab.Debugger.getScriptSource({ scriptId: script.scriptId })).scriptSource
            }));
        }

        // state now: all scripts parsed, paused at beginning of script
        return new ChromeDebugSession(tab, tabHandle, chromeScripts, this);
    },

    toString() {
        return "Chrome";
    }
};

export class ChromeDebugSession extends DebugSession {
    constructor(private readonly tab: any,
                private readonly tabHandle: any,
                readonly scripts: Script[],
                readonly browser: Browser) {
        super(browser, scripts);
        assert(scripts.length > 0, "chrome: tab has no scripts");
    }

    async close(): Promise<void> {
        await ChromeImpl.Close({ id: this.tabHandle.id });
    }

    async startExecution(): Promise<void> {
        await this.tab.Debugger.resume();
    }

    async setBreakpoint(location: Location): Promise<Breakpoint[]> {
        // TODO decide which gives less false positives: setting all breakpoints in a line
        // vs. setting only the first breakpoint in a line.
        // setting all breakpoints in a line matches Firefox semantics better for, e.g., for-loops
        // but is different, e.g., with console.log() (where there are 2 possible breakpoints in Ch)
        const breakpoint = await this.tab.Debugger.setBreakpoint({
            location: {
                scriptId: location.script.id,
                lineNumber: location.lineNumber
            }
        });
        breakpoint.id = breakpoint.breakpointId;
        // .actualLocation is already given by Chrome
        this.breakpoints_.push(breakpoint);
        return [breakpoint];

        // // returned locations contains precise column numbers
        // const { locations: possibleLocations } = await this.tab.Debugger.getPossibleBreakpoints({
        //     start: { scriptId: location.script.id, lineNumber: location.lineNumber },
        //     end: { scriptId: location.script.id, lineNumber: location.lineNumber + 1 },
        // });
        //
        // if (possibleLocations.length === 0) {
        //     // do sliding by not giving column explicitly
        //     possibleLocations.push({
        //         scriptId: location.script.id,
        //         lineNumber: location.lineNumber
        //     });
        // }
        //
        // let breakpoints: Breakpoint[] = [];
        // for (let location of possibleLocations) {
        //     let breakpoint = await this.tab.Debugger.setBreakpoint({ location });
        //     breakpoint.id = breakpoint.breakpointId;
        //     breakpoints.push(breakpoint);
        //     this.breakpoints_.push(breakpoint);
        // }
        // return breakpoints;
    }

    async removeBreakpoint(bp: Breakpoint): Promise<boolean> {
        assert(remove(this.breakpoints_, bp),
            "chrome: could not find breakpoint-to-remove in internal list of breakpoints");
        await this.tab.Debugger.removeBreakpoint({
            breakpointId: (bp as any).breakpointId
        });
        return true;
    }

    async resume(): Promise<void> {
        await this.tab.Debugger.resume();
    }

    async stepIn(): Promise<void> {
        await this.tab.Debugger.stepInto();
    }

    async stepOut(): Promise<void> {
        await this.tab.Debugger.stepOut();
    }

    async stepOver(): Promise<void> {
        await this.tab.Debugger.stepOver();
    }

    onNotRunning(callback: (pauseState: NotRunningState) => void): void {
        this.tab.Debugger.paused((chromePauseState: any) => {
            let pauseState = chromePauseState;
            pauseState.hitBreakpoints = chromePauseState.hitBreakpoints;
            pauseState.location = chromePauseState.callFrames[0].location;
            callback(pauseState);
        });
        this.tab.Page.loadEventFired(() => {
            callback("finished");
        });
    }

    async callStackFunctionNames(pauseState: NotRunningState): Promise<string[]> {
        const { callFrames } = (pauseState as any);
        return Promise.resolve(callFrames.map(f => f.functionName).filter(s => s.length > 0));
    }

    private static currentScopes(pauseState: NotRunningState): any[] {
        // NOTE slice(0, 1) is like headOption: empty array if empty before, else one-element array
        return (pauseState as any).callFrames.slice(0, 1).flatMap(callFrame => callFrame.scopeChain);
    }

    // NOTE a Scope is an object with the bindings in it as its properties
    private async scopePropertyDescriptors(scope): Promise<any[]> {
        return (await this.tab.Runtime.getProperties({
            objectId: scope.object.objectId,
            ownProperties: true
        })).result;
    }

    async nonGlobalVariables(pauseState: NotRunningState): Promise<Variable[]> {
        const scopes = ChromeDebugSession.currentScopes(pauseState).filter(scope => scope.type !== "global");
        const propertyDescriptors = (await Promise.all(scopes.map(
            async scope => await this.scopePropertyDescriptors(scope)
        ))).flatMap(id);
        const variables = propertyDescriptors.map(propertyDescriptorToBinding)
        // filter out arguments object
            .filter(binding => !(binding.name === "arguments" && binding.type === "array"));
        return Variable.takeFirst(variables);
    }
}

// see https://chromedevtools.github.io/debugger-protocol-viewer/1-2/Runtime/#type-RemoteObjectId
// and https://chromedevtools.github.io/debugger-protocol-viewer/1-2/Runtime/#type-PropertyDescriptor
function propertyDescriptorToBinding(propertyDescriptor): Variable {
    const binding = new Variable(
        propertyDescriptor.name,
        propertyDescriptor.value.type);

    // specify type more accurately for some subtypes of object
    if (propertyDescriptor.value.type === "object") {
        if (propertyDescriptor.value.subtype === "array") {
            binding.type = "array";
        } else if (propertyDescriptor.value.subtype === "null") {
            binding.type = "null";
        }
    }

    // add value only for primitives
    if (binding.type !== "object" && binding.type !== "function" && binding.type !== "array") {
        binding["value"] = (propertyDescriptor.value.unserializableValue !== undefined)
            // Infinity, -0, NaN are not serializable as JSON, so use this instead
            ? propertyDescriptor.value.unserializableValue
            : propertyDescriptor.value.value;

        // limit string output length to 30
        if (typeof binding["value"] === "string") {
            binding["value"] = binding["value"].slice(0, 30);
        }
    }
    return binding;
}
