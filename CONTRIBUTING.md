<!-- omit in toc -->
# Contributing to Beluga

First off, thanks for taking the time to contribute! ❤️

All types of contributions are encouraged and valued. See the [Table of Contents](#table-of-contents) for different ways to help and details about how this project handles them. Please make sure to read the relevant section before making your contribution. It will make it a lot easier for us maintainers and smooth out the experience for all involved. The community looks forward to your contributions. 🎉

> And if you like the project, but just don't have time to contribute, that's fine. There are other easy ways to support the project and show your appreciation, which we would also be very happy about:
> - Star the project
> - Tweet about it
> - Refer this project in your project's readme
> - Mention the project at local meetups and tell your friends/colleagues

<!-- omit in toc -->
## Table of Contents

- [I Have a Question](#i-have-a-question)
- [I Want To Contribute](#i-want-to-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Requesting Features](#requesting-features)
  - [Your First Code Contribution](#your-first-code-contribution)
    - [A Primer On Project Workflow](#a-primer-on-project-workflow)
    - [How Do I Submit a Good Code Contribution?](#how-do-i-submit-a-good-code-contribution)
- [Styleguides](#styleguides)

## I Have a Question

> If you want to ask a question, we assume that you have read the available [Documentation](https://github.com/Ekumen-OS/beluga).

Before opening a new issue to ask for help, it is best to check for already existing [Issues](https://github.com/Ekumen-OS/beluga/issues) that might be relevant to your question. In case you have found a suitable issue and still need clarification, you can write your question in there. It is also advisable to search the internet for answers first.

If you then still feel the need to ask a question and need clarification, we recommend the following:

- Open an [Issue](https://github.com/Ekumen-OS/beluga/issues/new).
- Provide as much context as you can about what you're running into.
- Provide project and platform versions, depending on what seems relevant.

We will try to answer back as soon as possible.

## I Want To Contribute

> ### Legal Notice <!-- omit in toc -->
> When contributing to this project, you must agree that you have authored 100% of the content,
> that you have the necessary rights to the content and that the content you contribute may be
> provided under the [Apache 2.0 License](./LICENSE) (item 5).

### Reporting Bugs

<!-- omit in toc -->
#### Before Submitting a Bug Report

A good bug report shouldn't leave others needing to chase you up for more information. Therefore, we ask you to investigate carefully, collect information, and describe the issue in detail in your report. Please complete the following steps in advance to help us fix any potential bug as fast as possible.

- Make sure that you are using the latest version.
- Determine if your bug is really a bug and not an error on your side e.g. using incompatible versions (Make sure that you have read the [documentation](https://github.com/Ekumen-OS/beluga). If you are looking for support, you might want to check [this section](#i-have-a-question)).
- To see if other users have experienced (and potentially already solved) the same issue you are having, check if there is not already a bug report existing for your bug or error in the [bug tracker](https://github.com/Ekumen-OS/beluga/issues?q=label%3Abug).
- Collect information about the bug:
  - Stack trace (Traceback)
  - OS, Platform and Version (Windows, Linux, macOS, x86, ARM)
  - Version of the interpreter, compiler, middleware, package manager, depending on what is relevant.
  - Possibly your input and the output
  - Can you reliably reproduce the issue? And can you also reproduce it with older versions?

<!-- omit in toc -->
#### How Do I Submit a Good Bug Report?

We use GitHub issues to track bugs and errors. If you run into an issue with the project:

- Open an [Issue](https://github.com/Ekumen-OS/beluga/issues/new). (Since we can't be sure at this point whether it is a bug or not, we ask you not to talk about a bug yet and not to label the issue.)
- Explain the behavior you would expect and the actual behavior.
- Please provide as much context as possible and describe the *reproduction steps* that someone else can follow to recreate the issue on their own. This usually includes your code. For good bug reports you should isolate the problem and create a reduced test case.
- Provide the information you collected in the previous section.

After the issue has been submitted:

- The project team will label the issue accordingly.
- A team member will try to reproduce the issue with the steps you provided. If no reproducible sequence of steps was provided or if the provided sequence does not seem to trigger the issue being reported, the team will ask you for further information about this and they will label the issue as `needs-repro`. Bugs with the `needs-repro` label will not be addressed until they've been successfully reproduced.
- If the team is able to reproduce the issue it will be labeled `needs-fix`, as well as possibly other labels, and the issue will be left to be [fixed by someone](#your-first-code-contribution).

### Requesting Features

This section guides you through submitting a feature request for Beluga, **including completely new additions and minor improvements to existing functionality**. Following these guidelines will help maintainers and the community to understand your suggestion and find related suggestions.

<!-- omit in toc -->
#### Before Submitting a Feature Request

- Make sure that you are using the latest version.
- Read the [documentation](https://github.com/Ekumen-OS/beluga) carefully and find out if the functionality is already covered.
- Perform a [search](https://github.com/Ekumen-OS/beluga/issues) to see if the feature has already been requested. If it has, add a comment to the existing issue instead of opening a new one.
- Find out whether your idea fits with the scope and aims of the project. It's up to you to make a strong case to convince the project's developers of the merits of this feature.

<!-- omit in toc -->
#### How Do I Submit a Good Feature Request?

Feature requests are tracked as [GitHub issues](https://github.com/Ekumen-OS/beluga/issues).

- Use a **clear and descriptive title** for the issue to identify the suggestion.
- Provide a **step-by-step description of the suggested feature** in as many details as possible.
- **Describe the current behavior** and **explain which behavior you expected to see instead** and why.
- **Explain why this feature would be useful** to most Beluga users. You may also want to point out the other projects that solved it better and which could serve as inspiration.

### Your First Code Contribution

This section guides you through contributing code to Beluga. Following these guidelines will help maintainers and ensure your contributions are reviewed and eventually accepted.

#### A Primer On Project Workflow

This projects adopts a [feature branch workflow](https://about.gitlab.com/topics/version-control/what-is-git-workflow/#feature-branching-git-workflow) ([forking workflow](https://about.gitlab.com/topics/version-control/what-is-git-workflow/#forking-git-workflow) for contributors that are not maintainers), with pull requests for code integration. Every code contribution must be associated to a [feature request](#requesting-features) or [bug report](#reporting-bugs) with enough consensus and evidence to move forward, signaled with `needs-work` and `needs-fix` labels respectively. Contributors must sign-off each commit by adding a `Signed-off-by: ...` line to every commit message to certify that they have the right to submit the code they are contributing to the project according to the [Developer Certificate of Origin (DCO)](https://developercertificate.org/). Pull requests must have been reviewed and approved at least once to be merged.

#### How Do I Submit a Good Code Contribution?

1. **Fork the [Beluga repository](https://github.com/Ekumen-OS/beluga/) to your GitHub account**.
1. **Clone the repository fork locally**. You will need `git`.
   ```bash
   git clone --recursive git@github.com:<your_username>/beluga.git
   ```
1. **Install pre-commit hooks**. You will need [`pre-commit`](https://pre-commit.com).
   ```bash
   cd beluga
   pre-commit install
   ```
1. **Create a new branch where your work will go**.
   ```bash
   git checkout -b <your_username>/fix-issue-12345 main
   ```
1. **Work on your contribution**. See [instructions](DEVELOPING.md) on how to get started with Beluga development.
1. **Test your changes**. For bug fixes, make sure regression tests are included.
1. **Document your changes as needed**. For new features, make sure added functionality is clearly documented.
1. **Push the branch to your fork on GitHub**.
1. **Open a pull request**. Make sure all tests and linters pass on your branch before opening.

## Styleguides

<!-- omit in toc -->
### Commit messages

Commit messages should adhere to the following good practices:

- Limit the subject line to 50 characters.
- Capitalize the subject line.
- Do not end the subject line with a period.
- Use the imperative mood in the subject line.
- Wrap the body at 72 characters.
- Use the body to explain _what_ and _why_ vs. _how_.

See https://cbea.ms/git-commit for more information and the reasoning behind this.

<!-- omit in toc -->
## Attribution
This guide is based on the **contributing-gen**. [Make your own](https://github.com/bttger/contributing-gen)!
