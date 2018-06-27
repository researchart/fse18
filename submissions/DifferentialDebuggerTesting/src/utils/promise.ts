export function ignoreArguments(...ignored: any[]): void {
}

/**
 * Convert a function that takes a callback to a Promise that resolves when the callback is called
 */
export function toPromise<E>(fn: (callback: (event: E) => void) => void): Promise<E> {
    return new Promise((resolve, reject) => fn(resolve));
}

/**
 * Promise that is rejected after some time.
 * NOTE will never be resolved.
 * NOTE can be used with Promse.race([...]) to timeout another Promise.
 * @param ms amount of time before rejection
 * @param reason rejection reason to give
 * @return {Promise<void>}
 */
export function timeout(ms: number, reason: string): Promise<void> {
    return new Promise<void>((resolve, reject) => {
        setTimeout(() => reject(reason), ms);
    });
}

export function delay(ms: number): Promise<void> {
    return new Promise<void>((resolve, reject) => {
        setTimeout(() => resolve(), ms)
    })
}