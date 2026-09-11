/**
 * HoloLib documentation enhancements (loaded on the Doxygen pages).
 *
 * Progressive enhancement only -- the docs are fully readable without this.
 * Reveals each content block with a gentle fade/slide as it scrolls into view.
 * The dedicated landing page (site root) handles the big title treatment, so
 * the docs pages just get the subtle reveal. Skipped under reduced-motion.
 */
(function () {
    "use strict";

    function setupReveal(container) {
        if (!("IntersectionObserver" in window)) return;
        if (window.matchMedia("(prefers-reduced-motion: reduce)").matches) return;

        var blocks = container.querySelectorAll(
            ":scope > h1, :scope > h2, :scope > h3, :scope > p, :scope > ul, " +
            ":scope > ol, :scope > table, :scope > div.fragment, :scope > pre, " +
            ":scope > blockquote, :scope > div.image, :scope > dl"
        );
        if (!blocks.length) return;

        var io = new IntersectionObserver(function (entries) {
            entries.forEach(function (entry) {
                if (entry.isIntersecting) {
                    entry.target.classList.add("holo-in");
                    io.unobserve(entry.target);
                }
            });
        }, { threshold: 0.1, rootMargin: "0px 0px -7% 0px" });

        blocks.forEach(function (el) {
            el.classList.add("holo-reveal");
            io.observe(el);
        });
    }

    function init() {
        var contents = document.querySelector("div.contents");
        if (!contents) return;
        var body = contents.querySelector(".textblock") || contents;
        setupReveal(body);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", init);
    } else {
        init();
    }
})();
