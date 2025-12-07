import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureList() {
  return [
    {
      title: 'Comprehensive Curriculum',
      description: (
        <>
          From foundational concepts in Physical AI to advanced humanoid robotics,
          our textbook covers everything you need to build intelligent embodied systems.
        </>
      ),
    },
    {
      title: 'Hands-On Learning',
      description: (
        <>
          Learn by doing with practical code examples, simulations in ROS 2 and Isaac Sim,
          and real-world robotics projects throughout each module.
        </>
      ),
    },
    {
      title: 'Industry-Standard Tools',
      description: (
        <>
          Master the tools used by leading robotics companies: ROS 2, NVIDIA Isaac Sim,
          Digital Twins, and Vision-Language-Action (VLA) models for robotics.
        </>
      ),
    },
  ];
}

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList().map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Comprehensive university-level textbook covering Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />

        <section className={styles.moduleOverview}>
          <div className="container">
            <h2 className="text--center margin-bottom--lg">Course Modules (13 Weeks)</h2>
            <div className="row">
              {/* Module 0 */}
              <div className="col col--4">
                <div className="card margin-bottom--lg">
                  <div className="card__header">
                    <h3>Module 0: Foundations</h3>
                    <span className="badge badge--primary">Weeks 1-2</span>
                  </div>
                  <div className="card__body">
                    <p>
                      Introduction to Physical AI, embodied intelligence, and robotics fundamentals.
                      <strong> 3 chapters</strong>
                    </p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-0-foundations">
                      Explore Module 0
                    </Link>
                  </div>
                </div>
              </div>

              {/* Module 1 */}
              <div className="col col--4">
                <div className="card margin-bottom--lg">
                  <div className="card__header">
                    <h3>Module 1: Kinematics & Control</h3>
                    <span className="badge badge--success">Weeks 3-5</span>
                  </div>
                  <div className="card__body">
                    <p>
                      Forward/inverse kinematics, motion planning, PID control, and advanced control theory.
                      <strong> 5 chapters</strong>
                    </p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-1-ros2">
                      Explore Module 1
                    </Link>
                  </div>
                </div>
              </div>

              {/* Module 2 */}
              <div className="col col--4">
                <div className="card margin-bottom--lg">
                  <div className="card__header">
                    <h3>Module 2: Digital Twin</h3>
                    <span className="badge badge--info">Weeks 6-7</span>
                  </div>
                  <div className="card__body">
                    <p>
                      Simulation with Gazebo, physics engines, sim-to-real transfer, and system identification.
                      <strong> 5 chapters</strong>
                    </p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-2-digital-twin">
                      Explore Module 2
                    </Link>
                  </div>
                </div>
              </div>

              {/* Module 3 */}
              <div className="col col--4">
                <div className="card margin-bottom--lg">
                  <div className="card__header">
                    <h3>Module 3: NVIDIA Isaac</h3>
                    <span className="badge badge--warning">Weeks 8-10</span>
                  </div>
                  <div className="card__body">
                    <p>
                      Isaac Sim, synthetic data generation, reinforcement learning, and edge deployment.
                      <strong> 5 chapters</strong>
                    </p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-3-isaac">
                      Explore Module 3
                    </Link>
                  </div>
                </div>
              </div>

              {/* Module 4 */}
              <div className="col col--4">
                <div className="card margin-bottom--lg">
                  <div className="card__header">
                    <h3>Module 4: VLA & Humanoids</h3>
                    <span className="badge badge--danger">Weeks 11-12</span>
                  </div>
                  <div className="card__body">
                    <p>
                      Vision-language-action models, humanoid platforms, and whole-body control.
                      <strong> 3 chapters</strong>
                    </p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/module-4-vla-humanoids">
                      Explore Module 4
                    </Link>
                  </div>
                </div>
              </div>

              {/* Capstone */}
              <div className="col col--4">
                <div className="card margin-bottom--lg">
                  <div className="card__header">
                    <h3>Capstone Project</h3>
                    <span className="badge badge--secondary">Week 13</span>
                  </div>
                  <div className="card__body">
                    <p>
                      Build a complete VLA-powered autonomous humanoid assistant integrating all modules.
                      <strong> 1 chapter</strong>
                    </p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--primary button--block"
                      to="/docs/capstone">
                      Start Capstone
                    </Link>
                  </div>
                </div>
              </div>
            </div>

            {/* Quick Stats */}
            <div className="row margin-top--lg">
              <div className="col col--12">
                <div className="hero hero--dark">
                  <div className="container">
                    <h3 className="text--center">Textbook Statistics</h3>
                    <div className="row">
                      <div className="col col--3 text--center">
                        <h2>21</h2>
                        <p>Total Chapters</p>
                      </div>
                      <div className="col col--3 text--center">
                        <h2>22</h2>
                        <p>Code Examples</p>
                      </div>
                      <div className="col col--3 text--center">
                        <h2>120+</h2>
                        <p>Glossary Terms</p>
                      </div>
                      <div className="col col--3 text--center">
                        <h2>68</h2>
                        <p>Citations</p>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
