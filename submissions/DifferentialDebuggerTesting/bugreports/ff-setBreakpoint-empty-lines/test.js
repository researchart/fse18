/*
 * In non-code lines, setBreakpoint requests will return a "slided" actualLocation. 
 * This is perfectly fine. However, if one then (after some breakpoints already slided
 * to the actualLocation) requests another breakpoint, now at the actualLocation, 
 * it will slide to some later line instead of staying at the expected line.
 */
// E.g. first set breakpoint in this or the empty line afterwards

console.log("then requests a breakpoint here")

// then it will end up actually past the last line (!?)