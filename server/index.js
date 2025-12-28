import express from 'express';
import cors from 'cors';
import dotenv from 'dotenv';
dotenv.config();

const app = express();
const PORT = process.env.PORT || 5000;

app.use(cors());
app.use(express.json());
app.use(express.static("public"));

// 👇 Add this route
app.get("/", (req, res) => {
  res.sendFile("index.html", { root: "public" });
});

console.log("Gemini Key loaded:", !!process.env.GEMINI_API_KEY);

// ===== BOOK CONTENT =====
const BOOK_CONTENT = `
Physical AI & Humanoid Robotics
This book explains Physical AI, embodied intelligence,
ROS 2, simulation, perception, navigation and VLA systems.
`;

app.post("/api/chat", async (req, res) => {
  try {
    const { message } = req.body;

    if (!message) {
      return res.json({ reply: "Message missing" });
    }

    const prompt = `
Use ONLY the following book content to answer.

BOOK:
${BOOK_CONTENT}

QUESTION:
${message}

RULE:
If answer is not in the book, reply exactly:
"This topic is not covered in the current version of the book."
`;

    console.log("Sending prompt to Gemini...");

    const response = await fetch(
      `https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent?key=${process.env.GEMINI_API_KEY}`,
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          contents: [
            {
              role: "user",
              parts: [{ text: prompt }],
            },
          ],
        }),
      }
    );

    const data = await response.json();

    console.log("Gemini response:", data);

    let reply =
      "This topic is not covered in the current version of the book.";

    if (data?.candidates?.[0]?.content?.parts?.[0]?.text) {
      reply = data.candidates[0].content.parts[0].text;
    }

    res.json({ reply });
  } catch (err) {
    console.error("Gemini API error:", err);
    res.json({
      reply: "⚠️ Sorry, the AI service is currently unavailable. Please try again.",
    });
  }
});

app.listen(PORT, () =>
  console.log(`✅ Server running at http://localhost:${PORT}`)
);
