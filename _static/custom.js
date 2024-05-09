var observer = new MutationObserver(function(mutations) {
  const dark = document.documentElement.dataset.theme == 'dark';
  if (dark) {
    document.documentElement.classList.add('dark-mode');
    document.documentElement.classList.remove('light-mode');
  } else {
    document.documentElement.classList.remove('dark-mode');
    document.documentElement.classList.add('light-mode');
  }
})
observer.observe(
    document.documentElement,
    {attributes: true, attributeFilter: ['data-theme']});
