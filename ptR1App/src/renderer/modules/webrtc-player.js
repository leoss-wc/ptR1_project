/**
 * A class to handle the WebRTC connection to a MediaMTX WHEP endpoint.
 */
export class WebRTCPlayer {
    /**
     * @param {string} url The WHEP endpoint URL.
     * @param {HTMLVideoElement} videoElement The video element to display the stream.
     * @param {HTMLElement} statusElement The element to display connection status.
     */
    constructor(url, videoElement, statusElement) {
        this.url = url;
        this.videoElement = videoElement;
        this.statusElement = statusElement;
        this.peerConnection = null;
    }

    /**
     * Starts the WebRTC connection process.
     */
    async connect() {
        this._updateStatus('Initializing WebRTC connection...');
        
        if (this.peerConnection) {
            this.disconnect();
        }

        // 1. Create a new RTCPeerConnection
        this.peerConnection = new RTCPeerConnection();

        // 2. Handle incoming tracks
        this.peerConnection.ontrack = (event) => {
            this._updateStatus('Stream connected!');
            console.log(`Track received: ${event.track.kind}`);
            if (!this.videoElement.srcObject) {
                this.videoElement.srcObject = new MediaStream();
            }
            this.videoElement.srcObject.addTrack(event.track);
        };
        
        // 3. Define the media we want to receive
        this.peerConnection.addTransceiver('video', { direction: 'recvonly' });
        // this.peerConnection.addTransceiver('audio', { direction: 'recvonly' }); // Uncomment for audio

        try {
            // 4. Create and set local SDP offer
            const offer = await this.peerConnection.createOffer();
            await this.peerConnection.setLocalDescription(offer);
            this._updateStatus('Sending offer to server...');

            // 5. Send offer to MediaMTX WHEP endpoint
            const response = await fetch(this.url, {
                method: 'POST',
                headers: { 'Content-Type': 'application/sdp' },
                body: this.peerConnection.localDescription.sdp
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Server responded with status ${response.status}: ${errorText}`);
            }

            // 6. Receive and set remote SDP answer
            this._updateStatus('Received answer, establishing connection...');
            const answerSdp = await response.text();
            await this.peerConnection.setRemoteDescription(
                new RTCSessionDescription({ type: 'answer', sdp: answerSdp })
            );

            console.log('WebRTC connection established successfully!');

        } catch (error) {
            console.error('Failed to start WebRTC stream:', error);
            this._updateStatus(`Error: ${error.message}`);
            this.disconnect(); // Clean up on failure
        }
    }

    /**
     * Disconnects the WebRTC connection.
     */
    disconnect() {
        if (this.peerConnection) {
            this.peerConnection.close();
            this.peerConnection = null;
            this._updateStatus('Disconnected.');
            console.log('WebRTC connection closed.');
        }
    }

    /**
     * Private helper to update the status element.
     * @param {string} message The message to display.
     */
    _updateStatus(message) {
        if (this.statusElement) {
            this.statusElement.innerText = message;
        }
    }
}
