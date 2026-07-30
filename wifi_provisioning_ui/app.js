const elements = {
  activeConnection: document.getElementById("active-connection"),
  interfaceAddress: document.getElementById("interface-address"),
  provisioningAccess: document.getElementById("provisioning-access"),
  apCard: document.getElementById("ap-card"),
  apRobotStationary: document.getElementById("ap-robot-stationary"),
  useApButton: document.getElementById("use-ap-button"),
  apMessage: document.getElementById("ap-message"),
  facilityCard: document.getElementById("facility-card"),
  facilityForm: document.getElementById("facility-form"),
  facilityMessage: document.getElementById("facility-message"),
  scanButton: document.getElementById("scan-button"),
  networkOptions: document.getElementById("network-options"),
  ssid: document.getElementById("ssid"),
  security: document.getElementById("security"),
  passwordField: document.getElementById("password-field"),
  password: document.getElementById("password"),
  showPassword: document.getElementById("show-password"),
  hiddenNetwork: document.getElementById("hidden-network"),
  stageButton: document.getElementById("stage-button"),
  switchCard: document.getElementById("switch-card"),
  stagedSsid: document.getElementById("staged-ssid"),
  forgetButton: document.getElementById("forget-button"),
  robotStationary: document.getElementById("robot-stationary"),
  switchButton: document.getElementById("switch-button"),
  switchMessage: document.getElementById("switch-message"),
  pendingCard: document.getElementById("pending-card"),
  pendingSsid: document.getElementById("pending-ssid"),
  confirmLink: document.getElementById("confirm-link"),
  rollbackCountdown: document.getElementById("rollback-countdown"),
  confirmCard: document.getElementById("confirm-card"),
  centralComputer: document.getElementById("central-computer"),
  confirmButton: document.getElementById("confirm-button"),
  confirmMessage: document.getElementById("confirm-message"),
  completeCard: document.getElementById("complete-card"),
  completeMessage: document.getElementById("complete-message"),
  robotAddress: document.getElementById("robot-address"),
  robotSubnet: document.getElementById("robot-subnet"),
  centralAddress: document.getElementById("central-address"),
  rosDomain: document.getElementById("ros-domain"),
  windowsConfigLink: document.getElementById("windows-config-link"),
};

const state = {
  status: null,
  countdownTimer: null,
  confirmationToken: new URLSearchParams(window.location.search).get("confirm") || "",
};

document.addEventListener("DOMContentLoaded", () => {
  elements.facilityForm.addEventListener("submit", stageFacilityNetwork);
  elements.apRobotStationary.addEventListener("change", () => {
    elements.useApButton.disabled = !(
      elements.apRobotStationary.checked && state.status?.can_provision
    );
  });
  elements.useApButton.addEventListener("click", useCurrentHotspot);
  elements.scanButton.addEventListener("click", scanNetworks);
  elements.security.addEventListener("change", syncSecurity);
  elements.showPassword.addEventListener("click", togglePassword);
  elements.robotStationary.addEventListener("change", () => {
    elements.switchButton.disabled = !elements.robotStationary.checked;
  });
  elements.switchButton.addEventListener("click", activateFacilityNetwork);
  elements.forgetButton.addEventListener("click", forgetFacilityNetwork);
  elements.centralComputer.addEventListener("change", () => {
    elements.confirmButton.disabled = !elements.centralComputer.checked;
  });
  elements.confirmButton.addEventListener("click", confirmFacilityNetwork);
  syncSecurity();
  void refreshStatus();
});

async function api(path, options = {}) {
  const response = await fetch(path, {
    cache: "no-store",
    ...options,
    headers: {
      "Content-Type": "application/json",
      "X-IntelliTrolley-Provisioning": "1",
      ...(options.headers || {}),
    },
  });
  const body = await response.json();
  if (!response.ok) {
    throw new Error(body.error || `Request failed (${response.status})`);
  }
  return body;
}

async function refreshStatus() {
  try {
    state.status = await api("/api/status", { method: "GET", headers: {} });
    renderStatus();
  } catch (error) {
    setMessage(elements.facilityMessage, error.message || "Pi status unavailable.", true);
  }
}

function renderStatus() {
  const status = state.status || {};
  elements.activeConnection.textContent = status.active_connection || "Unavailable";
  elements.interfaceAddress.textContent = status.interface_address || "No IPv4 address";
  elements.provisioningAccess.textContent = status.can_provision
    ? "Recovery hotspot"
    : "Read-only";
  elements.apCard.classList.toggle("hidden", !status.can_provision);
  elements.useApButton.disabled = !(
    status.can_provision && elements.apRobotStationary.checked
  );

  elements.facilityForm.querySelectorAll("input, select, button").forEach((control) => {
    control.disabled = !status.can_provision;
  });
  elements.scanButton.disabled = !status.can_provision;
  syncSecurity();

  const staged = status.staged;
  elements.switchCard.classList.toggle("hidden", !staged || !status.can_provision);
  if (staged) {
    elements.stagedSsid.textContent = staged.ssid;
  }

  if (state.confirmationToken) {
    elements.confirmCard.classList.remove("hidden");
    elements.apCard.classList.add("hidden");
    elements.facilityCard.classList.add("hidden");
    elements.switchCard.classList.add("hidden");
  }

  if (status.last_result && !state.confirmationToken) {
    setMessage(elements.facilityMessage, status.last_result, false);
  }
}

async function useCurrentHotspot() {
  if (!elements.apRobotStationary.checked) {
    return;
  }
  elements.useApButton.disabled = true;
  setMessage(elements.apMessage, "Updating the Pi and Windows network handoff…", false);
  try {
    const body = await api("/api/use-ap", {
      method: "POST",
      body: JSON.stringify({ robot_stationary: true }),
    });
    renderNetworkCompletion(body);
    elements.apCard.classList.add("hidden");
    elements.facilityCard.classList.add("hidden");
    elements.switchCard.classList.add("hidden");
  } catch (error) {
    setMessage(elements.apMessage, error.message || "Hotspot setup failed.", true);
    elements.useApButton.disabled = !elements.apRobotStationary.checked;
  }
}

function syncSecurity() {
  const open = elements.security.value === "open";
  elements.passwordField.classList.toggle("hidden", open);
  elements.password.required = !open;
  if (open) {
    elements.password.value = "";
  }
}

function togglePassword() {
  const showing = elements.password.type === "text";
  elements.password.type = showing ? "password" : "text";
  elements.showPassword.textContent = showing ? "Show" : "Hide";
}

async function scanNetworks() {
  setMessage(elements.facilityMessage, "Scanning nearby Wi-Fi…", false);
  elements.scanButton.disabled = true;
  try {
    const body = await api("/api/networks", { method: "GET", headers: {} });
    elements.networkOptions.innerHTML = "";
    for (const network of body.networks || []) {
      const option = document.createElement("option");
      option.value = network.ssid;
      option.label = `${network.signal}% · ${network.security_label}`;
      option.dataset.security = network.security;
      elements.networkOptions.appendChild(option);
    }
    setMessage(
      elements.facilityMessage,
      body.networks?.length
        ? `Found ${body.networks.length} network${body.networks.length === 1 ? "" : "s"}.`
        : "No networks were returned. You can still type the SSID manually.",
      false,
    );
  } catch (error) {
    setMessage(
      elements.facilityMessage,
      `${error.message || "Scan failed"} You can type the SSID manually.`,
      true,
    );
  } finally {
    elements.scanButton.disabled = !state.status?.can_provision;
  }
}

async function stageFacilityNetwork(event) {
  event.preventDefault();
  setMessage(elements.facilityMessage, "Saving profile without switching…", false);
  elements.stageButton.disabled = true;
  try {
    const selectedOption = [...elements.networkOptions.options].find(
      (option) => option.value === elements.ssid.value,
    );
    if (selectedOption?.dataset.security === "enterprise") {
      throw new Error(
        "This network uses 802.1X/Enterprise authentication, which requires an administrator profile.",
      );
    }
    const body = await api("/api/stage", {
      method: "POST",
      body: JSON.stringify({
        ssid: elements.ssid.value,
        security: elements.security.value,
        password: elements.password.value,
        hidden: elements.hiddenNetwork.checked,
      }),
    });
    elements.password.value = "";
    setMessage(elements.facilityMessage, body.message, false);
    await refreshStatus();
  } catch (error) {
    setMessage(elements.facilityMessage, error.message || "Profile save failed.", true);
  } finally {
    elements.stageButton.disabled = !state.status?.can_provision;
  }
}

async function activateFacilityNetwork() {
  if (!elements.robotStationary.checked) {
    return;
  }
  elements.switchButton.disabled = true;
  setMessage(elements.switchMessage, "Scheduling the protected network switch…", false);
  try {
    const body = await api("/api/activate", {
      method: "POST",
      body: JSON.stringify({ robot_stationary: true }),
    });
    elements.pendingSsid.textContent = body.ssid;
    elements.confirmLink.href = body.confirm_url;
    elements.pendingCard.classList.remove("hidden");
    elements.switchCard.classList.add("hidden");
    setMessage(elements.switchMessage, body.message, false);
    startCountdown(body.deadline);
  } catch (error) {
    setMessage(elements.switchMessage, error.message || "Network switch failed.", true);
    elements.switchButton.disabled = !elements.robotStationary.checked;
  }
}

function startCountdown(deadlineSeconds) {
  if (state.countdownTimer !== null) {
    window.clearInterval(state.countdownTimer);
  }
  const render = () => {
    const remaining = Math.max(0, Math.ceil(deadlineSeconds - Date.now() / 1000));
    elements.rollbackCountdown.textContent = remaining > 0
      ? `Automatic rollback in ${remaining} seconds if not confirmed.`
      : "Confirmation expired. Reconnect to the IntelliTrolley hotspot.";
    if (remaining <= 0 && state.countdownTimer !== null) {
      window.clearInterval(state.countdownTimer);
      state.countdownTimer = null;
    }
  };
  render();
  state.countdownTimer = window.setInterval(render, 1000);
}

async function forgetFacilityNetwork() {
  if (!window.confirm("Forget the saved facility Wi-Fi profile?")) {
    return;
  }
  elements.forgetButton.disabled = true;
  try {
    const body = await api("/api/forget", {
      method: "POST",
      body: JSON.stringify({ confirm: true }),
    });
    setMessage(elements.facilityMessage, body.message, false);
    await refreshStatus();
  } catch (error) {
    setMessage(elements.switchMessage, error.message || "Could not forget profile.", true);
  } finally {
    elements.forgetButton.disabled = false;
  }
}

async function confirmFacilityNetwork() {
  if (!state.confirmationToken || !elements.centralComputer.checked) {
    return;
  }
  elements.confirmButton.disabled = true;
  setMessage(elements.confirmMessage, "Committing the facility connection…", false);
  try {
    const body = await api("/api/confirm", {
      method: "POST",
      body: JSON.stringify({
        token: state.confirmationToken,
        central_computer: true,
      }),
    });
    elements.confirmCard.classList.add("hidden");
    renderNetworkCompletion(body);
    window.history.replaceState({}, document.title, "/");
    state.confirmationToken = "";
  } catch (error) {
    setMessage(elements.confirmMessage, error.message || "Confirmation failed.", true);
    elements.confirmButton.disabled = !elements.centralComputer.checked;
  }
}

function renderNetworkCompletion(body) {
  elements.completeCard.classList.remove("hidden");
  elements.completeMessage.textContent = body.robot_defaults_updated
    ? body.message
    : `${body.message} The Pi ROS peer file could not be updated; use the Windows tool’s SSH step.`;
  elements.robotAddress.textContent = body.robot_address;
  elements.robotSubnet.textContent = body.robot_subnet;
  elements.centralAddress.textContent = body.central_address;
  elements.rosDomain.textContent = String(body.ros_domain_id);
  elements.windowsConfigLink.href = body.configuration_uri;
}

function setMessage(element, message, isError) {
  element.textContent = message || "";
  element.classList.toggle("error", Boolean(isError));
}
