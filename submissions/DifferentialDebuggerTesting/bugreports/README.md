# Found Debugger Bugs

| Browser | Short Description of the Bug | Bugtracker Link | Status |
| --- | --- | --- | --- |
| Chromium | Breakpoint slides over function and to next variable declaration instead | https://bugs.chromium.org/p/chromium/issues/detail?id=784852 | Fixed |
| Chromium | Inline breakpoint markers missing with multi-line ```for```-loop header | https://bugs.chromium.org/p/chromium/issues/detail?id=719912 | Fixed |
| Chromium | Inline breakpoint marker missing on ```for```-loop with ```var```-declared variable | https://bugs.chromium.org/p/chromium/issues/detail?id=752443 | Fixed |
| Chromium | ```let```/```const``` variables not shown before declaration in global scope | https://bugs.chromium.org/p/chromium/issues/detail?id=718827 | Reported |
| Chromium | Breakpoint in last line wraps around to first statement | https://bugs.chromium.org/p/chromium/issues/detail?id=730177 | Fixed |
| Firefox | Breakpoint sliding broken if previous breakpoint slided to same line | https://bugzilla.mozilla.org/show_bug.cgi?id=1343060 | Reported |
| Firefox | Cannot set breakpoint in ```for in```/```for of```-loop without loop variable declaration | https://bugzilla.mozilla.org/show_bug.cgi?id=1362416 | Fixed |
| Firefox | Cannot remove breakpoint set after last statement | https://bugzilla.mozilla.org/show_bug.cgi?id=1362439 | Reported |
| Firefox | ```let```/```const``` variables not shown on step out | https://bugzilla.mozilla.org/show_bug.cgi?id=1362428 | Confirmed |
| Firefox | ```let```/```const``` variables change value to ```null``` before reaching declaration | https://bugzilla.mozilla.org/show_bug.cgi?id=1362451 | Confirmed |
| Firefox | Pauses twice at single function call breakpoint | https://bugzilla.mozilla.org/show_bug.cgi?id=1370641 | Reported |
| Firefox | Single step in jumps over ```for of```-loop | https://bugzilla.mozilla.org/show_bug.cgi?id=1362403 | Fixed |
| Firefox | Number of steps at ```for```-loop depends on whitespace | https://bugzilla.mozilla.org/show_bug.cgi?id=1363325 | Reported |
| Firefox | Number of steps at ```while```-loop depends on whitespace | https://bugzilla.mozilla.org/show_bug.cgi?id=1370655 | Reported |
| Firefox | ```let```-bound variable shown with wrong value from previous scope | https://bugzilla.mozilla.org/show_bug.cgi?id=1363328 | Confirmed |
| Firefox | Function parameter with same name has wrong value | https://bugzilla.mozilla.org/show_bug.cgi?id=1362432 | Reported |
| Firefox | Debugger pauses at last statement, even if dead code | https://bugzilla.mozilla.org/show_bug.cgi?id=1370648 | Fixed |
| Firefox | Two steps needed to show return value at closing brace | https://bugzilla.mozilla.org/show_bug.cgi?id=923975, then moved to https://github.com/devtools-html/debugger.html/issues/3390 | (Already reported independently) Fixed |
| Firefox | Cannot set breakpont at some operators like && and === | https://bugzilla.mozilla.org/show_bug.cgi?id=1417196 | Reported |
| Firefox | Breakpoint is hit again after a step, line seems to be executed twice | https://bugzilla.mozilla.org/show_bug.cgi?id=1417240, then moved to https://github.com/devtools-html/debugger.html/issues/4013 | Fixed |
