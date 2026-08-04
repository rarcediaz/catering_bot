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
  networkResultsField: document.getElementById("network-results-field"),
  networkResults: document.getElementById("network-results"),
  ssid: document.getElementById("ssid"),
  security: document.getElementById("security"),
  passwordField: document.getElementById("password-field"),
  password: document.getElementById("password"),
  showPassword: document.getElementById("show-password"),
  hiddenNetwork: document.getElementById("hidden-network"),
  stageButton: document.getElementById("stage-button"),
  savedProfilesCard: document.getElementById("saved-profiles-card"),
  savedProfiles: document.getElementById("saved-profiles"),
  profilesMessage: document.getElementById("profiles-message"),
  switchCard: document.getElementById("switch-card"),
  stagedSsid: document.getElementById("staged-ssid"),
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
  elements.networkResults.addEventListener("change", selectScannedNetwork);
  elements.security.addEventListener("change", syncSecurity);
  elements.showPassword.addEventListener("click", togglePassword);
  elements.robotStationary.addEventListener("change", () => {
    elements.switchButton.disabled = !elements.robotStationary.checked;
  });
  elements.switchButton.addEventListener("click", activateFacilityNetwork);
  elements.savedProfiles.addEventListener("click", handleSavedProfileAction);
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
  const transitionActive = Boolean(
    status.transition_phase && status.transition_phase !== "idle",
  );
  elements.activeConnection.textContent = status.active_connection || "Unavailable";
  elements.interfaceAddress.textContent = status.interface_address || "No IPv4 address";
  elements.provisioningAccess.textContent = transitionActive
    ? "Wi-Fi change pending"
    : status.can_provision
      ? status.using_recovery_ap
        ? "Recovery hotspot"
        : "Active Wi-Fi"
      : "Read-only";
  elements.apCard.classList.toggle(
    "hidden",
    !status.can_configure_ap,
  );
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
  renderSavedProfiles(status.saved_profiles || [], status.can_provision);
  if (transitionActive || state.confirmationToken) {
    elements.savedProfilesCard.classList.add("hidden");
  }

  if (state.confirmationToken) {
    elements.confirmCard.classList.remove("hidden");
    elements.apCard.classList.add("hidden");
    elements.facilityCard.classList.add("hidden");
    elements.switchCard.classList.add("hidden");
  }

  if (transitionActive && !state.confirmationToken) {
    setMessage(
      elements.facilityMessage,
      status.last_result || "A Wi-Fi recovery operation is in progress. Controls will return when it finishes.",
      false,
    );
  } else if (!status.can_provision && !state.confirmationToken) {
    setMessage(
      elements.facilityMessage,
      "Open this page from a computer on the same active Wi-Fi network as the Pi to make changes.",
      true,
    );
  } else if (status.last_result && !state.confirmationToken) {
    setMessage(elements.facilityMessage, status.last_result, false);
  }
}

function renderSavedProfiles(profiles, canProvision) {
  elements.savedProfiles.innerHTML = "";
  elements.savedProfilesCard.classList.toggle("hidden", !profiles.length);
  for (const profile of profiles) {
    const row = document.createElement("div");
    row.className = "profile-row";

    const summary = document.createElement("div");
    const name = document.createElement("strong");
    name.textContent = profile.ssid;
    const status = document.createElement("span");
    status.textContent = profile.active
      ? "Active"
      : profile.confirmed
        ? "Confirmed · automatic"
        : "Saved · not confirmed";
    summary.append(name, status);

    const actions = document.createElement("div");
    actions.className = "profile-actions";
    const selectButton = document.createElement("button");
    selectButton.type = "button";
    selectButton.className = "secondary compact";
    selectButton.textContent = profile.staged ? "Selected" : "Select";
    selectButton.dataset.action = "select";
    selectButton.dataset.uuid = profile.uuid;
    selectButton.disabled = !canProvision || profile.active || profile.staged;

    const removeButton = document.createElement("button");
    removeButton.type = "button";
    removeButton.className = "text-button compact";
    removeButton.textContent = "Remove";
    removeButton.dataset.action = "remove";
    removeButton.dataset.uuid = profile.uuid;
    removeButton.dataset.ssid = profile.ssid;
    removeButton.disabled = !canProvision || profile.active;
    actions.append(selectButton, removeButton);
    row.append(summary, actions);
    elements.savedProfiles.appendChild(row);
  }
}

async function handleSavedProfileAction(event) {
  const button = event.target.closest("button[data-action]");
  if (!button || button.disabled) {
    return;
  }
  const profileUuid = button.dataset.uuid || "";
  if (button.dataset.action === "select") {
    await selectSavedProfile(profileUuid);
  } else if (button.dataset.action === "remove") {
    await removeSavedProfile(profileUuid, button.dataset.ssid || "this network");
  }
}

async function selectSavedProfile(profileUuid) {
  setMessage(elements.profilesMessage, "Selecting saved profile…", false);
  try {
    const body = await api("/api/select", {
      method: "POST",
      body: JSON.stringify({ uuid: profileUuid }),
    });
    setMessage(elements.profilesMessage, body.message, false);
    await refreshStatus();
  } catch (error) {
    setMessage(elements.profilesMessage, error.message || "Could not select profile.", true);
  }
}

async function removeSavedProfile(profileUuid, ssid) {
  if (!window.confirm(`Remove the saved Wi-Fi profile for ${ssid}?`)) {
    return;
  }
  setMessage(elements.profilesMessage, `Removing ${ssid}…`, false);
  try {
    const body = await api("/api/forget", {
      method: "POST",
      body: JSON.stringify({ uuid: profileUuid, confirm: true }),
    });
    setMessage(elements.profilesMessage, body.message, false);
    await refreshStatus();
  } catch (error) {
    setMessage(elements.profilesMessage, error.message || "Could not remove profile.", true);
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
    elements.networkResults.innerHTML = "";
    for (const network of body.networks || []) {
      const option = document.createElement("option");
      option.value = network.ssid;
      option.label = `${network.signal}% · ${network.security_label}`;
      option.dataset.security = network.security;
      elements.networkOptions.appendChild(option);

      const visibleOption = document.createElement("option");
      visibleOption.value = network.ssid;
      visibleOption.textContent = `${network.ssid} — ${network.signal}% — ${network.security_label}`;
      visibleOption.dataset.security = network.security;
      elements.networkResults.appendChild(visibleOption);
    }
    elements.networkResultsField.classList.toggle(
      "hidden",
      !body.networks?.length,
    );
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

function selectScannedNetwork() {
  const option = elements.networkResults.selectedOptions[0];
  if (!option) {
    return;
  }
  elements.ssid.value = option.value;
  if (option.dataset.security === "enterprise") {
    setMessage(
      elements.facilityMessage,
      `${option.value} uses Enterprise authentication and cannot be provisioned here.`,
      true,
    );
    return;
  }
  elements.security.value = option.dataset.security === "open" ? "open" : "wpa-psk";
  syncSecurity();
  setMessage(elements.facilityMessage, `Selected ${option.value}.`, false);
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
    startCountdown(body.timeout_s);
  } catch (error) {
    setMessage(elements.switchMessage, error.message || "Network switch failed.", true);
    elements.switchButton.disabled = !elements.robotStationary.checked;
  }
}

function startCountdown(timeoutSeconds) {
  if (state.countdownTimer !== null) {
    window.clearInterval(state.countdownTimer);
  }
  // The Pi and this computer may have different wall clocks. Measure the
  // returned duration entirely on this browser's clock.
  const deadlineMilliseconds = Date.now() + timeoutSeconds * 1000;
  const render = () => {
    const remaining = Math.max(
      0,
      Math.ceil((deadlineMilliseconds - Date.now()) / 1000),
    );
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
