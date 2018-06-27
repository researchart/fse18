import { remove } from "./utils";

/**
 * Similar to other EventEmitters but events are full objects instead of strings/symbols. Listeners
 * are thus not fired on emit-ting a particular string, but when they match an "event template" they
 * are interested in.
 * NOTE see objectMatchesTemplate function for definition of "matching with a template".
 */
export class ObjectEventEmitter {

    private listeners: Array<(event: any) => boolean> = [];
    listenerCount(): number {
        return this.listeners.length;
    }

    /**
     * NOTE we can narrow the argument type of the listener to T and one can thus (safely) use the
     * properties of the event that were specified in the eventTemplate.
     * @return {ObjectListener} the internal representation of the listener, to be able to manually
     * remove it with removeListener.
     */
    addListener<T>(eventTemplate: T, listener: (event: T) => void): ObjectListener {
        const objectListener = ObjectEventEmitter.ifMatchingListener(eventTemplate, listener);
        this.listeners.push(objectListener);
        return objectListener;
    }

    /**
     * @return {ObjectListener} the internal representation of the listener, to be able to manually
     * remove it with removeListener.
     */
    addListenerOnce<T>(eventTemplate: T, listener: (event: T) => void): ObjectListener {
        const objectListener = ObjectEventEmitter.ifMatchingListener(eventTemplate, listener);
        const onceObjectListener = this.onceListener(objectListener);
        this.listeners.push(onceObjectListener);
        return onceObjectListener;
    }

    /**
     * @param objectListener NOTE must be the (internal) object returned by addListener/addListenerOnce,
     * not the handler the user gave to register.
     * @return {boolean} true if the listener existed and was removed.
     */
    removeListener(objectListener: ObjectListener): boolean {
        return remove(this.listeners, objectListener);
    }

    emit(event: any): void {
        // dispatch event to all listeners, they will return early if not interested
        this.listeners.forEach(listener => listener(event));
    }

    /**
     * Utility function that wraps a listener, such that it is only executed if the event matches
     * the eventTemplate.
     */
    private static ifMatchingListener<T>(eventTemplate: T, listener: (T) => void): ObjectListener {
        function ifMatchingListenerImpl(event: any): boolean {
            if (objectMatchesTemplate(event, eventTemplate)) {
                listener(event);
                return true;
            }
            return false;
        }

        return ifMatchingListenerImpl;
    }

    /**
     * Utility function that wraps an objectListener, such that it removes itself after being
     * executed (i.e. when the event matches its eventTempalte).
     * @param objectListener NOTE how this is an ObjectListener, i.e. a listener that informs us
     * if it fired or not.
     */
    private onceListener(objectListener: ObjectListener): ObjectListener {
        // NOTE otherwise this in onceListenerImpl below is undefined
        const self: ObjectEventEmitter = this;

        function onceListenerImpl(event: any): boolean {
            const wasExecuted = objectListener(event);
            if (wasExecuted) {
                self.removeListener(onceListenerImpl);
            }
            return wasExecuted;
        }

        return onceListenerImpl;
    }

}

/**
 * Internal representation of an object listener.
 * @return {boolean} true if it handled the event, false if did not (because it didn't match its
 * event template).
 */
export interface ObjectListener {
    (event: any): boolean;
}

/**
 * NOTE uses TypeScript user defined type guards/type predicate return type
 * (see https://www.typescriptlang.org/docs/handbook/advanced-types.html).
 * @return {boolean} true if all properties of template are present in objectToTest and all truthy,
 * non-array properties have the same value.
 * NOTE the objectToTest may very well have more properties than the template.
 */
function objectMatchesTemplate<T1, T2>(objectToTest: T1, template: T2): objectToTest is T1 & T2 {
    return Object.getOwnPropertyNames(template).every(
        (propertyName) => {
            // the object must have all properties that the template has
            return objectToTest.hasOwnProperty(propertyName)
                && (
                    // for truthy values it has to have the same value and type
                    objectToTest[propertyName] === template[propertyName]
                    // falsey template values are not checked
                    || !template[propertyName]
                    // and also not arrays
                    || template[propertyName] instanceof Array
                );
        }
    );
}
