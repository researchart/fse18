interface Array<T> {
    flatMap<R>(callbackfn: (value: T) => R[]): R[];
}

Array.prototype.flatMap = function (callbackfn) {
    return Array.prototype.concat.apply([], this.map(callbackfn));
};
