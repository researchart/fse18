import { assertNever } from "./utils";

export function randomIntegerInRange(rng: any, min: number, maxExclusive: number): number {
    min = Math.ceil(min);
    maxExclusive = Math.floor(maxExclusive);
    return Math.floor(rng() * (maxExclusive - min)) + min;
}

export function randomTrue(rng: any, probability: number): boolean {
    return rng() < probability;
}

export function shuffle<T>(rng: any, array: T[]): T[] {
    // see https://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle#The_modern_algorithm
    const shuffledArray = array.slice();

    for (let currentIndex = 0; currentIndex < shuffledArray.length; currentIndex++) {
        const randomIndex = randomIntegerInRange(rng, currentIndex, shuffledArray.length);

        // swap the random remaining element with the current
        const temp = shuffledArray[currentIndex];
        shuffledArray[currentIndex] = shuffledArray[randomIndex];
        shuffledArray[randomIndex] = temp;
    }

    return shuffledArray;
}

export function randomArrayElement<T>(rng: any, array: T[]): T {
    return array[randomIntegerInRange(rng, 0, array.length)];
}

export function randomElement<T>(rng: any, iterable: Iterable<T>): T {
    return randomArrayElement(rng, [...iterable]);
}

export function randomElements<T>(rng, values: T[], count: number): T[] {
    let result: T[] = [];
    for (let i = 0; i < count; i++) {
        const element = values[randomIntegerInRange(rng, 0, values.length)];
        result.push(element);
    }
    return result;
}

export function randomElementWithProbability<T>(rng: any, values: Array<[T, number]>): T | null {
    // see http://stackoverflow.com/a/9330667

    // normalize sum of probabilities to 1
    const normalization = values.reduce((acc, elem) => acc + elem[1], 0);
    // if no value is possible, don't generate anything
    if (normalization == 0.0) {
        return null;
    }
    const selector = rng() * normalization;
    let cumulativeProbability = 0.0;
    for (let [value, probability] of values) {
        cumulativeProbability += probability;
        if (selector <= cumulativeProbability) {
            return value;
        }
    }

    return assertNever();
}
