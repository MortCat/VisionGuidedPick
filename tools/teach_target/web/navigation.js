(() => {
    'use strict';

    const shell = document.body;
    const button = document.querySelector('.app-navigation-toggle');
    const links = document.getElementById('workspace-navigation-links');
    const narrow = window.matchMedia('(max-width: 800px)');
    const preferenceKey = 'ptc.navigation.expanded';

    let desktopExpanded = true;
    let narrowExpanded = false;

    function render() {
        const expanded = narrow.matches ? narrowExpanded : desktopExpanded;
        if (!expanded && links.contains(document.activeElement)) button.focus();
        shell.dataset.navigation = expanded ? 'expanded' : 'collapsed';
        button.setAttribute('aria-expanded', String(expanded));
        links.hidden = !expanded;
    }

    function restorePreference() {
        try {
            const stored = localStorage.getItem(preferenceKey);
            if (stored !== null) desktopExpanded = stored === 'true';
        } catch {
            // The button remains usable when storage is unavailable.
        }
        narrowExpanded = false;
        render();
    }

    button.hidden = false;
    restorePreference();
    window.addEventListener('pageshow', restorePreference);
    narrow.addEventListener('change', () => {
        narrowExpanded = false;
        render();
    });

    button.addEventListener('click', () => {
        if (narrow.matches) {
            narrowExpanded = !narrowExpanded;
        } else {
            desktopExpanded = !desktopExpanded;
            try {
                localStorage.setItem(preferenceKey, String(desktopExpanded));
            } catch {
                // Keep the preference in memory for this page.
            }
        }
        render();
    });

    document.addEventListener('click', event => {
        if (!narrow.matches || !narrowExpanded || button.parentElement.contains(event.target)) return;
        narrowExpanded = false;
        render();
    });

    document.addEventListener('keydown', event => {
        if (event.key !== 'Escape' || !narrow.matches || !narrowExpanded) return;
        if (button.parentElement.contains(document.activeElement)) event.preventDefault();
        narrowExpanded = false;
        render();
    });
})();
