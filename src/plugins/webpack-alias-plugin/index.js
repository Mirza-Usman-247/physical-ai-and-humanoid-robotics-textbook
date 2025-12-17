const path = require('path');

module.exports = function (context) {
  return {
    name: 'webpack-alias-plugin',
    configureWebpack(config, isServer, content) {
      config.resolve = config.resolve || {};
      config.resolve.alias = config.resolve.alias || {};
      config.resolve.alias['@'] = path.resolve(context.siteDir, 'src');

      return config;
    },
  };
};