const express = require('express');
const cors = require('cors');

const app = express();
app.use(express.json());
app.use(cors());

// In-memory state for motor telemetry and commands
let motorState = {
    status: "OFF",
    phase1: 0.0,
    phase2: 0.0,
    phase3: 0.0,
    lastUpdated: null
};

let pendingCommand = null; // "ON" or "OFF" or null

// 1. Endpoint for ESP32 to send telemetry (currents & status)
app.post('/api/telemetry', (req, res) => {
    const { status, phase1, phase2, phase3 } = req.body;
    motorState = {
        status: status || "OFF",
        phase1: phase1 || 0.0,
        phase2: phase2 || 0.0,
        phase3: phase3 || 0.0,
        lastUpdated: new Date()
    };
    res.json({ success: true, pendingCommand });
    if (pendingCommand) {
        pendingCommand = null;
    }
});

// 2. Endpoint for Vercel Frontend to fetch motor telemetry
app.get('/api/data', (req, res) => {
    res.json(motorState);
});

// 3. Endpoint for Vercel Frontend to send START/STOP command
app.post('/api/control', (req, res) => {
    const { state } = req.body; // "ON" or "OFF"
    if (state === "ON" || state === "OFF") {
        pendingCommand = state;
        res.json({ success: true, message: `Command ${state} queued for ESP32` });
    } else {
        res.status(400).json({ error: "Invalid state. Use 'ON' or 'OFF'." });
    }
});

// 4. Endpoint for ESP32 to check for pending commands
app.get('/api/command', (req, res) => {
    res.json({ command: pendingCommand });
    pendingCommand = null;
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
    console.log(`Smart Starter Backend running on port ${PORT}`);
});
