if (typeof mermaid !== "undefined") {
  mermaid.initialize({
    startOnLoad: false,
    securityLevel: "loose",
    theme: "neutral",
  });
}

const renderMermaid = () => {
  if (typeof mermaid === "undefined") {
    return;
  }
  mermaid.run({ querySelector: ".mermaid" });
};

if (typeof document$ !== "undefined") {
  document$.subscribe(renderMermaid);
} else {
  window.addEventListener("load", renderMermaid);
}
