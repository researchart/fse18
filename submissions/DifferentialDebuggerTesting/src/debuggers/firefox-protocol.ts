import log = require("winston");
import WebSocket = require("ws");
import { ObjectEventEmitter } from "../utils/object-event-emitter";
import { ignoreArguments, timeout } from "../utils/promise";

// log.level = "debug";

export interface PausedState extends ServerPacket {
    type: "paused";
    actor: string;
    frame: {
        actor: string;
        // type: string;
        where: {
            source: Source;
            line: number;
            column: number;
        }
    };
    poppedFrames: any[];
    why: {
        type: string;
        actors: any[];
    };
}

export interface Tab {
    actor: string;
    title: string;
    url: string;
    // ... much more
}

export class Thread {
    constructor(readonly actor: string) {
    }
}

export interface Source {
    actor: string;
    url: string;
    // ... much more
}

/*
 * internal interfaces
 */

interface LongString {
    actor: string;
    initial: string;
    length: number;
}
function isLongString(x: any): x is LongString {
    return x.hasOwnProperty("type") && x.type === "longString";
}

interface ServerPacket {
    from: string;
}

interface ClientPacket {
    to: string;
    type: string;
}

export class FirefoxRemoteProtocol {

    static async connect(host: string = "localhost", port: number = 6080, initTimeoutMs?: number): Promise<FirefoxRemoteProtocol> {
        const debuggerUrl = `ws://${host}:${port}`;
        const ff = new FirefoxRemoteProtocol(new WebSocket(debuggerUrl), new ObjectEventEmitter());

        // connect via websockets, message dispatch to own listeners
        ff.socket.on("open", () => {
            log.debug("connected to " + debuggerUrl);
        });
        ff.socket.on("close", (code, reason) => {
            log.debug(`connection closed with code ${code} and reason '${reason}'`);
        });
        ff.socket.on("message", (data, flags) => {
            if (flags.binary) {
                log.warn("ignoring incoming binary packet");
                return;
            }
            const packet = JSON.parse(data);

            if (packet.error !== undefined) {
                log.warn("incoming error packet: " + JSON.stringify(packet, null, 2));
            } else {
                log.debug("incoming packet: " + JSON.stringify(packet, null, 2));
            }

            ff.eventEmitter.emit(packet);
        });

        // wait for server hello until some timeout
        const init = ff.once("root", { applicationType: "browser" }).then(ignoreArguments);
        if (initTimeoutMs !== undefined) {
            await Promise.race([
                init,
                timeout(initTimeoutMs, `server hello timed out in ${initTimeoutMs}ms`)
            ]);
        } else {
            await init;
        }

        return ff;
    }

    /*
     * Concrete requests
     */

    async navigateTo(tab: Tab, url: string): Promise<void> {
        await Promise.all([
            this.request(tab.actor, "navigateTo", {}, { url }),
            // FIXME Firefox doesn't always send this message (?), then this blocks indefinitely!?
            this.once("root", { type: "tabListChanged" })
        ]);
    }

    async listTabs(): Promise<Tab[]> {
        return (await this.request("root", "listTabs", { tabs: [] })).tabs;
    }

    async attachAndPause(tab: Tab): Promise<Thread> {
        const { threadActor } = await this.request(tab.actor, "attach", {
            type: "tabAttached",
            threadActor: ""
        });
        await this.request(threadActor, "attach", { type: "paused" });
        return new Thread(threadActor);
    }

    async detachAndRun(tab: Tab): Promise<void> {
        await this.request(tab.actor, "detach", { type: "detached" });
    }

    async sources(thread: Thread): Promise<Source[]> {
        const reply = await this.request(thread.actor, "sources", { sources: [] });
        return reply.sources;
    }

    async getSource(source: Source): Promise<string> {
        const maybeLongString = await this.request(source.actor, "source", { source: "" });
        return this.resolveString(maybeLongString.source);
    }

    private resolveString(str: string | LongString): Promise<string> {
        if (isLongString(str)) {
            return this.request(str.actor, "substring", { substring: "" }, {
                start: 0,
                end: str.length
            }).then((response) => response.substring);
        } else {
            return Promise.resolve(str);
        }
    }

    /**
     * Generic method to request and expect a particular response.
     * @param toActor
     * @param type
     * @param replyProperties properties to expect on the reply, as matched with the
     * ObjectEventEmitter's objectMatchingTemplate function.
     * @param options additional options to send with the request
     * @return {Promise<ServerPacket & T>} resolved when a reply with replyProperties is received,
     * rejected when an error from this toActor is received.
     */
    request<T>(toActor: string, type: string, replyProperties: T, options = {}): Promise<ServerPacket & T> {
        const packet = Object.assign({ to: toActor, type: type }, options);
        this.send(packet);
        return this.once(toActor, replyProperties);
    }

    /**
     * Generic method to act on an unsolicited server packet (like for events)
     * @param fromActor
     * @param packetProperties properties to expect on the packet, as matched with the
     * ObjectEventEmitter's objectMatchingTemplate function.
     * @param callback
     * @return {Promise<void>} Resolved when the FirefoxRemoteProtocol connection has been correctly established.
     */
    on<T>(fromActor: string, packetProperties: T, callback: (packet: ServerPacket & T) => void): void {
        const replyTemplate = Object.assign({ from: fromActor }, packetProperties);
        this.eventEmitter.addListener(replyTemplate, callback);
    }

    once<T>(fromActor: string, packetProperties: T): Promise<ServerPacket & T> {
        const replyTemplate = Object.assign({ from: fromActor }, packetProperties);
        return new Promise((resolve, reject) => {
            this.eventEmitter.addListenerOnce(replyTemplate, resolve);
        });
    }

    close(): void {
        log.debug(`closing, still having ${this.eventEmitter.listenerCount()} active listeners`);
        this.socket.close();
    }

    private constructor(private readonly socket: WebSocket,
                        private readonly eventEmitter: ObjectEventEmitter) {
    }

    private send(packet: ClientPacket): void {
        log.debug("send packet: " + JSON.stringify(packet, null, 2));
        this.socket.send(JSON.stringify(packet), { compress: false, binary: false });
    }

}
