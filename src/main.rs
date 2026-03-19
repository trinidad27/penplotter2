use std::{sync::{atomic::{AtomicU64, Ordering}, Arc}};
use axum::{
    extract::{ws::{Message, WebSocket, WebSocketUpgrade}, State},
    http::{HeaderMap, StatusCode},
    response::IntoResponse,
    routing::{any, get, post},
    Router,
};
use futures_util::{sink::SinkExt, stream::{StreamExt}};
use serde::{Deserialize, Serialize};
use tokio::sync::mpsc::{self, UnboundedReceiver};
use base64::{engine::general_purpose::URL_SAFE, Engine};
use dashmap::DashMap;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "SCREAMING_SNAKE_CASE")]
enum ClientMessage {
    Submit {
        session_id: u64,
        svg: String
    }
}

#[derive(Serialize, Clone)]
#[serde(tag = "type", rename_all = "SCREAMING_SNAKE_CASE")]
enum ServerMessage {
    StartSession {
        session_id: u64
    },
    Turn {
        session_id: u64
    },
    Draw {
        session_id: u64,
        svg: String
    },
    Error
}

#[derive(Clone, PartialEq, Eq)]
enum ConnectedDeviceState {
    Idle,
    InSession(u64)
}

#[derive(Clone)]
struct ConnectedDevice {
    account_id: u64,
    tx_chan: mpsc::UnboundedSender<ServerMessage>,
    state: ConnectedDeviceState
}

#[derive(Clone)]
enum SessionState {
    Pending,
    Active,
    Ended
}

#[derive(Clone)]
struct Session {
    id: u64,
    device_a: u64,
    device_b: u64,
    state: SessionState,
    turn_device: u64
}

#[derive(Clone)]
struct AppState {
    // If using device_conns and sessions at the same time, always lock sessions first
    // to prevent deadlocks
    device_conns: Arc<DashMap<u64, ConnectedDevice>>,
    sessions: Arc<DashMap<u64, Session>>,
    next_sid: Arc<AtomicU64>
}

fn authenticate_device(device_token: &[u8]) -> Option<(u64, u64)> {
    // TODO: do actual lookup, probably in a DB
    if device_token[0] == 0 {
        Some((1, 1))
    } else if device_token[0] == 1 {
        Some((2, 2))
    } else {
        None
    }
}

async fn ws_upgrade_handler(ws: WebSocketUpgrade, State(state): State<AppState>, headers: HeaderMap) -> impl IntoResponse {
    let Some(auth_header) = headers.get("Authorization") else {
        return (StatusCode::BAD_REQUEST, "Missing authorization").into_response();
    };

    let Ok(token) = auth_header.to_str() else {
        return (StatusCode::UNAUTHORIZED, "Invalid Token").into_response();
    };
    
    let Ok(token_decoded) = URL_SAFE.decode(token) else {
        return (StatusCode::BAD_REQUEST, "Invalid Token").into_response();
    };

    let Some((device_id, account_id)) = authenticate_device(&token_decoded[..]) else {
        return (StatusCode::UNAUTHORIZED, "Unauthorized").into_response();
    };

    if state.device_conns.contains_key(&device_id) {
        // already connected, reject connection
        return (StatusCode::CONFLICT, "Already connected on another websocket").into_response();
    };

    let (tx, rx) = mpsc::unbounded_channel::<ServerMessage>();
    state.device_conns.insert(device_id, ConnectedDevice {
        account_id: account_id,
        tx_chan: tx,
        state: ConnectedDeviceState::Idle
    });

    ws.on_upgrade(move |socket| handle_ws(socket, state, rx, device_id))
}

fn get_other(device: u64, device_a: u64, device_b: u64) -> u64 {
    if device == device_a {
        device_b
    } else {
        device_a
    }
}

fn handle_client_message(msg: &ClientMessage, device_id: u64, app_state: &AppState) {
    match msg {
        ClientMessage::Submit { session_id, svg } => {
            // Lookup session_id
            let dev = app_state.device_conns.get(&device_id).unwrap();

            let ConnectedDeviceState::InSession(sid) = dev.state else {
                return // error here
            };

            if sid != *session_id {
                return; // another error here
            }

            let mut session = app_state.sessions.get_mut(&sid).unwrap();

            let other_device = get_other(device_id, session.device_a, session.device_b);
            session.turn_device = other_device;

            send_to_device(app_state, other_device, ServerMessage::Draw { session_id: sid, svg: svg.clone() });
            send_to_device(app_state, other_device, ServerMessage::Turn { session_id: sid });
        }
    }
}

async fn handle_ws(socket: WebSocket, app_state: AppState, mut rx_chan: UnboundedReceiver<ServerMessage>, device_id: u64) {
    let (mut sender, mut receiver) = socket.split();

    tokio::spawn(async move {
        while let Some(msg) = rx_chan.recv().await {
            let json = serde_json::to_string(&msg).unwrap();
            if sender.send(json.into()).await.is_err() {
                break;
            }
        }
    });

    while let Ok(msg) = receiver.next().await.unwrap() {
        match msg {
            Message::Text(text) => {
                let parsed = match serde_json::from_str::<ClientMessage>(&text) {
                    Ok(p) => p,
                    Err(_) => {
                        // TODO: handle invalid message
                        continue;
                    }
                };
                handle_client_message(&parsed, device_id, &app_state);
            },

            Message::Close(_) => break,

            // TODO: handle invalid messages (Binary)
            _ => {}
        }
    }
    
    app_state.device_conns.remove(&device_id);
}

fn send_to_device(state: &AppState, device_id: u64, msg: ServerMessage) {
    let Some(dev) = state.device_conns.get(&device_id) else {
        return;
    };

    dev.tx_chan.send(msg).unwrap();
}

fn try_start_session(state: &AppState, session_id: u64) {
    let (dev_a, dev_b, turn_dev) = {
        let Some(session) = state.sessions.get(&session_id) else {
            return;
        };

        (session.device_a, session.device_b, session.turn_device)
    };


    // Setup device A
    {
        let Some(mut dev) = state.device_conns.get_mut(&dev_a) else {
            return;
        };

        if dev.state != ConnectedDeviceState::Idle {
            return;
        }

        dev.state = ConnectedDeviceState::InSession(session_id);
    }

    let revert = |state: &AppState, dev_id: u64| {
        if let Some(mut dev) = state.device_conns.get_mut(&dev_id) {
            dev.state = ConnectedDeviceState::Idle;
        };
    };

    // Setup device B, or revert A
    {
        match state.device_conns.get_mut(&dev_b) {
            Some(mut dev) => {
                if dev.state != ConnectedDeviceState::Idle {
                    revert(state, dev_a);
                    return;
                }

                dev.state = ConnectedDeviceState::InSession(session_id);
            }

            None => {
                revert(state, dev_a);
                return;
            }
        }
    }


    // Update session state
    {
        let Some(mut session) = state.sessions.get_mut(&session_id) else {
            revert(state, dev_a);
            revert(state, dev_b);
            return;
        };
        session.state = SessionState::Active;
    }


    let start = ServerMessage::StartSession { session_id: session_id };
    send_to_device(state, dev_a, start.clone());
    send_to_device(state, dev_b, start);
    
    send_to_device(state, turn_dev, ServerMessage::Turn { session_id: session_id });
}

async fn start_session_handler(State(state): State<AppState>) -> impl IntoResponse {
    // Pretend that we have looked up two users and got their device ids,
    // and also assume these devices are online
    let device_a = 1;
    let device_b = 2;
    let sid = state.next_sid.fetch_add(1, Ordering::Relaxed);

    state.sessions.insert(sid, Session {
        id: sid,
        device_a: device_a,
        device_b: device_b,
        state: SessionState::Pending,
        turn_device: device_a
    });

    try_start_session(&state, sid);

    (StatusCode::OK, "ok")
}

#[tokio::main]
async fn main() {
    let state = AppState {
        device_conns: Arc::new(DashMap::new()),
        sessions: Arc::new(DashMap::new()),
        next_sid: Arc::new(AtomicU64::new(0))
    };

    let app = Router::new()
        .route("/ws", any(ws_upgrade_handler))
        .route("/dummy_start", post(start_session_handler)) // dummy route, fix later
        .with_state(state);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3000").await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

