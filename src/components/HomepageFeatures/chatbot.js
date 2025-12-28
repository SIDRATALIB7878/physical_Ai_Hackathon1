function toggleChat() {
  const chatWindow = document.getElementById("chatbot-window");
  chatWindow.style.display =
    chatWindow.style.display === "none" || chatWindow.style.display === ""
      ? "flex"
      : "none";
}

async function sendMessage() {
  const input = document.getElementById("userInput");
  const chatBox = document.getElementById("chatBox");

  const message = input.value.trim();
  if (!message) return;

  chatBox.innerHTML += `<div><b>You:</b> ${message}</div>`;
  input.value = "";

  try {
    const res = await fetch("http://localhost:5000/api/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ message })
    });

    const data = await res.json();
    chatBox.innerHTML += `<div><b>Bot:</b> ${data.reply}</div>`;
    chatBox.scrollTop = chatBox.scrollHeight;
  } catch (err) {
    chatBox.innerHTML += `<div>⚠️ Backend error</div>`;
  }
}
