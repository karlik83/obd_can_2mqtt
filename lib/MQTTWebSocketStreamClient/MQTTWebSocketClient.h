/*
 * This program is free software; you can use it, redistribute it
 * and / or modify it under the terms of the GNU General Public License
 * (GPL) as published by the Free Software Foundation; either version 3
 * of the License or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program, in a file called gpl.txt or license.txt.
 *  If not, write to the Free Software Foundation Inc.,
 *  59 Temple Place - Suite 330, Boston, MA  02111-1307 USA
 */
#pragma once

#include <ArduinoHttpClient.h>

class MQTTWebSocketClient : public WebSocketClient {
public:
    MQTTWebSocketClient(Client &client, const char *host, uint16_t port);

    int connect(const IPAddress &ip, uint16_t port);

    int begin(const char *aPath, const char *protocol = nullptr);

    int begin(const String &aPath, const char *protocol = nullptr);

    /**
     * Send a complete masked binary WebSocket frame in a single transport write.
     *
     * WebSocketClient::endMessage() emits the opcode byte, length byte(s), the
     * 4-byte mask and the payload as separate write() calls. Over the cellular
     * modem each becomes its own AT+CCHSEND / TLS record, and Mosquitto's
     * libwebsockets frame parser drops the connection when a frame header does
     * not arrive in one piece ("First packet not CONNECT"). Assembling the whole
     * frame here and writing it once avoids that.
     *
     * @param buf the payload
     * @param len the payload length
     * @return <code>0</code> on success
     */
    int sendBinaryFrame(const uint8_t *buf, size_t len);

    bool flush(unsigned int maxWaitMs = 0);

    bool stop(unsigned int maxWaitMs = 0);
};
